//
// Floating-Point Unit
// CPU-facing x87 top, using the 387 I/O port protocol (0xF8/0xFC)
//
module x87_unit #(
    parameter ENABLE_X87 = 0
)(
    input  logic        clk,
    input  logic        reset_n,

    input  logic        req_valid,          // Paging presents an x87 I/O cycle.
    input  logic        req_data_port,      // 0=f8 command/status; 1=fc operand data.
    input  logic        req_write,          // Direction of the CPU-side cycle.
    input  logic [3:0]  req_be,
    input  logic [31:0] req_wdata,
    output logic        req_accepted,       // Bridge captured the request.
    output logic        req_complete,       // Bridge completed the protocol action.
    output logic        req_read_complete,
    output logic [31:0] req_rdata,

    input  logic        direct_launch,      // Instruction issue checks the x87 overlays.
    input  logic        direct_candidate,   // Recipe is an eligible direct x87 form.
    input  logic        direct_allowed,     // CR0 and runtime policy permit x87 use.
    input  logic        launch_wait,        // The issuing instruction is WAIT (9B).
    input  logic        wait_allowed,       // CR0.MP and TS not both set (no CR0 write in flight).
    input  logic        wait_nop,           // No coprocessor: WAIT has nothing to wait for.
    input  logic [10:0] direct_fop,         // FOP of the executing instruction.
    input  logic [10:0] launch_fop,         // FOP of the issuing instruction (its x87 form).
    input  logic        operand_part,       // An X87_OPERAND word executed: OPR_R is next.
    input  logic        operand_post,       // An X87_POST word executed: OPR_R is the last part.
    input  logic        store_word,         // The store overlay's command/result word is current.
    input  logic        store_go,           // ...and its destination check completed (no memory stall).
    input  logic        store_exec,         // ...and executes.
    input  logic        next_part,          // An X87_NEXT word (a part's WR) executed.
    output logic        store_opr_commit,   // Write store_opr_data to OPR_R.
    output logic        store_record,       // A direct store records FIP/FCS:FOP/operand.
    output logic        store_hold,         // Hold the ROM at the store word until the result is in.
    output logic [31:0] store_opr_data,
    output logic        direct_taken,       // The issued x87 overlay went direct.
    output logic        load_commit,        // A direct load's operand entered the x87 queue.
    output logic        direct_reg_taken,   // ...as a register form (no operand).
    output logic        direct_active,      // One direct transport owns instruction issue.
    output logic        direct_mem_req,     // Launch fault-checked demand read through paging.
    output logic        direct_stall,       // Hold retirement until control accepts the operand.
    output logic        wait_rni,           // The WAIT word retires: an RNI for it.

    input  logic        mem_accepted,
    input  logic [1:0]  mem_addr_low,
    input  logic        mem_read_complete,
    input  logic        mem_servicing,
    input  logic [31:0] mem_rdata,
    input  logic [31:0] opr_r,              // OPR_R: a split m32 read, an operand part.
    input  logic        cancel,             // Fault, interrupt, or frontend squash.

    output logic        busy_n,
    output logic        pereq,
    output logic        error_n,
    output logic [31:0] debug_state
);

// The issuing x87 form, from its FOP ({opcode[2:0], ModR/M}). Only the
// optimizer's x87 overlays are candidates; these name which one runs.
// launch_fop of a WAIT is not an x87 form: the ESC paths below skip it.
wire        esc_candidate = direct_candidate && !launch_wait;
wire        l_reg = launch_fop[7:6] == 2'b11;
wire  [2:0] l_esc = launch_fop[10:8];          // D8 + l_esc
wire  [2:0] l_op  = launch_fop[5:3];           // ModR/M.reg
// Stores: FST/FSTP m32, FIST/FISTP m32, FST/FSTP m64 and FIST/FISTP m16
// (D9, DB, DD, DF /2 /3), FSTP m80 (DB /7), FBSTP (DF /6), FISTP m64 (DF /7).
wire        l_m80 = (l_op == 3'd7) ? (l_esc == 3'd3) : (l_op == 3'd6) && (l_esc == 3'd7);
wire        l_m64 = (l_op == 3'd7) ? (l_esc == 3'd7) : (l_esc == 3'd5) && (l_op[2:1] == 2'b01);
wire        l_store = !l_reg && ((l_esc[0] && (l_op[2:1] == 2'b01)) || l_m80 || l_m64);
wire  [1:0] l_parts = l_m80 ? 2'd3 : l_m64 ? 2'd2 : 2'd1;
// Status reads: FNSTSW AX (DF E0), FNSTSW m16 (DD /7), FNSTCW m16 (D9 /7).
// The FNSTSW AX overlay also takes DF E1-FF (DF /4-/7 register forms share
// its entry); those are not status reads here and fall back.
wire        l_status = (launch_fop == 11'h7e0) ||
                       (!l_reg && (l_op == 3'd7) && ((l_esc == 3'd1) || (l_esc == 3'd5)));
wire        l_regcmd = l_reg && !((l_esc == 3'd7) && l_op[2]);    // x87-reg forms
// FLDCW (D9 /5): its RD words hand in the control word (x87-fldcw).
wire        l_fldcw = !l_reg && (l_esc == 3'd1) && (l_op == 3'd5);
// Loads whose RD words hand in the operand (x87-m16/m64/m80-load): DC-DF
// memory forms and DB /5, and FLDCW; the m32 loads use direct_mem_req.
wire        l_operand = !l_reg && !l_store && !l_status &&
                        (l_esc[2] || ((l_esc[1:0] == 2'b11) && (l_op == 3'd5)) || l_fldcw);

logic        queue_safe;           // Masked-exception mode permits one queued command.
logic        mask_load_pending;    // ...and no mask load in flight can withdraw that.
logic        direct_ready;         // x87_control can capture the direct FOP/data pair.
logic        direct_issued_r;      // Demand read has crossed the paging handshake.
logic        direct_crossing_r;    // Dword spans two naturally aligned memory words.
logic        direct_data_valid_r;  // Complete fault-checked operand is buffered.
logic [31:0] direct_data_r;        // Buffered direct m32 operand.
wire         m32_valid = direct_active && direct_data_valid_r;
// Other loads (m16, m64, m80): the overlay's RD words read the operand and
// the word after each DLY marks OPR_R as its next part. OPR_R is sampled the
// cycle after the marker: no read of this instruction can write it sooner,
// and a DLY may retire before its data is in OPR_R. X87_POST marks the last
// part and posts FOP and operand from the cycle after; the overlay holds the
// instruction until the x87 accepts them. Parts are dwords, except the m16
// operand and the m80 tail, which are words: one part (m16) or three (m80)
// end in two bytes.
logic        op_taken_r;           // The issued x87 overlay reads its operand in microcode.
logic        op_ctrl_r;            // ...the control word (FLDCW): nothing to record.
logic        op_part_r;            // Sample OPR_R as the next part now.
logic        op_post_r;            // Post the load with OPR_R as its last part.
logic  [1:0] op_parts_r;           // Parts sampled before the last.
logic [31:0] op_part0_r, op_part1_r;
wire         direct_valid = m32_valid || op_post_r;
wire  [1:0]  direct_words = op_post_r ? op_parts_r + 2'd1 : 2'd1;
wire         direct_tail16 = op_post_r && (op_parts_r != 2'd1);
wire  [79:0] direct_data = !op_post_r ? {48'h0, direct_data_r} :
                           (op_parts_r == 2'd0) ? {48'h0, opr_r} :
                           (op_parts_r == 2'd1) ? {16'h0, opr_r, op_part0_r} :
                                                  {opr_r[15:0], op_part1_r, op_part0_r};
// Register forms: the FOP is posted to the bridge as a command write, in the
// executing instruction's first cycle. Taken when CR0 allows x87 use and no
// unmasked exception is pending, and either the 386 routine would post it at
// once (the x87 is idle or may queue one command, and the bridge is free), or
// the bridge is taken but will be free soon (queue_short: everything ahead of
// it is a running arithmetic operation and at most one short command) and
// every exception is masked and stays masked (no mask load in flight), so
// nothing queued ahead can raise one. Then the instruction holds in its first
// word (direct_stall) until the bridge takes the command. The routine would
// wait for BUSY# instead, taking interrupts while it waits: it still does,
// whenever the wait could be long.
logic        direct_reg_r;         // The issued instruction took the register path.
logic        dcmd_pending_r;       // ...and its command awaits bridge acceptance.
logic        dcmd_wait;            // ...still, this cycle: the instruction holds.
logic        queue_short;          // The bridge's command will be taken soon.
logic        posted_pending;       // The bridge holds a posted write.
logic        br_accepted, br_complete;
logic        br_idle;              // The bridge takes a request now.
logic        br_read_complete;
logic [31:0] br_rdata;
// Stores and status reads share the store word and the result path.
// Stores: the overlay checks the whole destination for writing first (CW
// words, so nothing after the command can fault). In the store word, post
// the FOP as a command, then take the result from the x87's transfer queue
// as it is produced (result_pop: no data-port read through the bridge), in
// the parts the 80386 routine reads and writes: one (m16, m32), two dwords
// (m64) or two dwords and the two-byte tail (m80). The store word holds
// until the last part is in; OPR_R takes the first part the cycle the hold
// ends (ST_DONE), and the WR word after the store word writes it to memory.
// A part's WR word is an X87_NEXT word: the cycle after it, OPR_R takes the
// next part for the next WR.
// Status reads post nothing: from issue (ST_READ) they wait until every
// earlier x87 command has completed, which is when the 386 routine's
// command write would be accepted, then take the status or control word as
// its f8 port read would. The store word holds until the word is in OPR_R.
typedef enum logic [1:0] {ST_IDLE, ST_CMD, ST_READ, ST_DONE} st_state_t;
st_state_t   st_state;
logic        direct_st_r;          // The issued instruction took the store or status path.
logic        st_status_r;          // ...a status read (FNSTSW, FNSTCW).
logic        st_control_r;         // ...of the control word (FNSTCW).
logic        x87_quiet;            // No x87 command in the bridge, queued or running.
logic        x87_quiet_r;          // ...nor in the x87 unit, last cycle.
logic [15:0] status_word, control_word;
logic  [1:0] st_parts_r;           // Result parts of the store.
logic  [1:0] st_taken_r;           // ...taken from the x87 so far.
logic [31:0] st_part0_r;           // The part for OPR_R...
logic [31:0] st_part1_r;           // ...and the parts after it.
logic [15:0] st_part2_r;
logic        st_commit_r;          // OPR_R takes st_part0_r: ST_DONE's first
                                   // cycle, and the cycle after an X87_NEXT word.
// WAIT (x87-wait): one word, X87_STORE, which holds in place (store_hold)
// until the x87 has completed every command before it, status included
// (x87_quiet, as the status reads wait). Then, with nothing to report, the
// word retires with an RNI of its own (wait_rni) and x87_direct_taken, a NOP
// from there on; with an unmasked exception pending it runs untaken, and the
// original routine (its fallback) reports the exception as it always has:
// #MF with CR0.NE, else FERR# (IRQ13) while it polls, until IGNNE#. The
// routine is also taken at once with CR0.MP and TS set (#NM) or a CR0 write
// beside the issue, and when the x87 is on a long operation: the word holds
// while the x87 runs plain arithmetic or a held command (queue_short, as the
// register-command wait), or for at most 32 consecutive cycles of anything
// else (a transcendental, FPREM, BCD, ...). The hold takes no interrupt;
// the routine polls BUSY# and takes interrupts, restarting the WAIT.
// Without a coprocessor WAIT is a NOP but for #NM.
logic        wait_hold_r;          // The issued WAIT's word holds: the x87 is not done.
logic        wait_go_r;            // ...or retires as a NOP.
logic  [4:0] wait_slow_r;          // Held cycles in a row the x87 ran no plain operation.
wire         wait_launch = direct_candidate && launch_wait;
wire         wait_quiet;           // Every command before the WAIT has completed.
wire         st_cmd = direct_st_r && store_go && !cancel &&
                      (st_state == ST_IDLE || st_state == ST_CMD);
wire         result_pop = (st_state == ST_READ) && !st_status_r;
logic        result_valid;
logic [31:0] result_data;
// The store's result comes whole when control converts it while the store
// waits (all but FBSTP): every part in one cycle, not through the queue.
wire         result_whole_take = direct_st_r && (st_state == ST_READ) && !st_status_r;
logic        result_whole_valid;
logic [79:0] result_whole_data;
wire         dcmd_sel = dcmd_pending_r || st_cmd;    // a command write (F8)
wire         own_cmd = dcmd_sel;
assign direct_taken = direct_active || direct_reg_r || direct_st_r || op_taken_r ||
                      wait_go_r;
assign wait_rni = wait_go_r;
assign direct_reg_taken = direct_reg_r;
wire         direct_release = direct_valid && direct_ready;
// FLDCW records no FIP/FCS:FOP or operand, as its original routine does not.
assign load_commit = direct_release && !op_ctrl_r;

assign direct_mem_req = direct_active && !direct_issued_r && !direct_data_valid_r;
assign store_hold = store_word && ((direct_st_r && (st_state != ST_DONE)) || wait_hold_r);
assign store_opr_commit = st_commit_r;
assign store_record = st_commit_r && (st_state == ST_DONE) && !st_status_r;
assign store_opr_data = st_part0_r;
// The register command is the bridge's request while it waits: it is taken
// exactly when the bridge is idle (registers only, nothing from the CPU's
// other requests).
assign dcmd_wait = dcmd_pending_r && !br_idle;
assign direct_stall = ((direct_active || op_post_r) && !direct_release) || store_hold ||
                      dcmd_wait;

generate
if (ENABLE_X87) begin : gen_x87
    wire        cmd_valid;
    wire [10:0] cmd_fop;
    wire        cmd_ready;
    wire        ctl_quiet;
    wire        word_in_valid;
    wire  [3:0] word_in_be;
    wire [31:0] word_in_data;
    wire        word_in_ready;
    wire        read_req_valid;
    wire        read_req_data_port;
    wire  [3:0] read_req_be;
    wire        read_req_ready;
    wire        read_resp_valid;
    wire [31:0] read_resp_data;
    wire        ctl_busy_n, ctl_pereq;
    // A posted port write is the x87's as soon as the bridge accepts it: until
    // it dispatches, report BUSY# and PEREQ as control does for an accepted
    // command, so the coprocessor-wait microcode samples a settled protocol.
    assign busy_n = ctl_busy_n && !posted_pending;
    assign pereq  = ctl_pereq || posted_pending;
    // control may take a command behind a running one: quiet is its own
    // report that every accepted command has completed, status included.
    assign x87_quiet = ctl_quiet && !cmd_valid && !posted_pending;
    assign wait_quiet = x87_quiet && !dcmd_pending_r;

    // The direct command has the bridge to itself: no ESC routine (the only
    // source of paging's x87 cycles) runs while it waits. Paging sees neither
    // its acceptance nor its completion.
    assign req_accepted = br_accepted && !dcmd_sel;
    assign req_complete = br_complete && !dcmd_sel;
    assign req_read_complete = br_read_complete && !dcmd_sel;
    assign req_rdata = br_rdata;
    x87_bridge bridge (
        .clk(clk), .reset(!reset_n),
        .req_valid(own_cmd || (!dcmd_sel && req_valid)),
        .req_data_port(own_cmd ? 1'b0 : req_data_port),
        .req_write(own_cmd || (!dcmd_sel && req_write)),
        .req_be(own_cmd ? 4'h3 : req_be),
        .req_wdata(own_cmd ? {21'd0, direct_fop} : req_wdata),
        .req_accepted(br_accepted), .req_complete(br_complete),
        .req_read_complete(br_read_complete), .req_rdata(br_rdata),
        .posted_pending(posted_pending), .idle(br_idle),
        .cmd_valid(cmd_valid), .cmd_fop(cmd_fop), .cmd_ready(cmd_ready),
        .word_in_valid(word_in_valid), .word_in_be(word_in_be),
        .word_in_data(word_in_data), .word_in_ready(word_in_ready),
        .read_req_valid(read_req_valid),
        .read_req_data_port(read_req_data_port), .read_req_be(read_req_be),
        .read_req_ready(read_req_ready), .read_resp_valid(read_resp_valid),
        .read_resp_data(read_resp_data)
    );

    x87_control control (
        .clk(clk), .reset(!reset_n),
        .cmd_valid(cmd_valid), .cmd_fop(cmd_fop), .cmd_ready(cmd_ready),
        .quiet(ctl_quiet),
        .direct_valid(direct_valid), .direct_fop(direct_fop),
        .direct_data(direct_data), .direct_words(direct_words),
        .direct_tail16(direct_tail16), .direct_ready(direct_ready),
        .word_in_valid(word_in_valid), .word_in_be(word_in_be),
        .word_in_data(word_in_data), .word_in_ready(word_in_ready),
        .read_req_valid(read_req_valid),
        .read_req_data_port(read_req_data_port), .read_req_be(read_req_be),
        .read_req_ready(read_req_ready), .read_resp_valid(read_resp_valid),
        .read_resp_data(read_resp_data),
        .result_pop(result_pop), .result_valid(result_valid),
        .result_data(result_data),
        .result_whole_take(result_whole_take),
        .result_whole_valid(result_whole_valid),
        .result_whole_data(result_whole_data),
        .status_out(status_word), .control_out(control_word),
        .busy_n(ctl_busy_n), .pereq(ctl_pereq), .error_n(error_n),
        .queue_safe(queue_safe), .mask_load_pending(mask_load_pending),
        .queue_short(queue_short),
        .debug_state(debug_state)
    );
end else begin : gen_no_x87
    assign req_accepted = 1'b0;
    assign req_complete = 1'b0;
    assign result_valid = 1'b0;
    assign result_data = 32'h0;
    assign result_whole_valid = 1'b0;
    assign result_whole_data = 80'h0;
    assign status_word = 16'h0;
    assign control_word = 16'h0;
    assign x87_quiet = 1'b0;
    assign wait_quiet = 1'b0;
    assign br_accepted = 1'b0;
    assign br_read_complete = 1'b0;
    assign br_rdata = 32'h0;
    assign br_complete = 1'b0;
    assign posted_pending = 1'b0;
    assign br_idle = 1'b1;
    assign req_read_complete = 1'b0;
    assign req_rdata = 32'h0;
    assign busy_n = 1'b1;
    assign pereq = 1'b0;
    assign error_n = 1'b1;
    assign queue_safe = 1'b0;
    assign mask_load_pending = 1'b0;
    assign queue_short = 1'b0;
    assign direct_ready = 1'b0;
    assign debug_state = 32'h8000_0000;
end
endgenerate

always_ff @(posedge clk) begin
    if (!reset_n) begin
        direct_st_r         <= 1'b0;
        st_state            <= ST_IDLE;
        st_status_r         <= 1'b0;
        st_control_r        <= 1'b0;
        x87_quiet_r         <= 1'b0;
        st_parts_r          <= 2'd1;
        st_taken_r          <= 2'd0;
        st_part0_r          <= 32'h0;
        st_part1_r          <= 32'h0;
        st_part2_r          <= 16'h0;
        st_commit_r         <= 1'b0;
        direct_reg_r        <= 1'b0;
        dcmd_pending_r      <= 1'b0;
        direct_active       <= 1'b0;
        direct_issued_r     <= 1'b0;
        direct_crossing_r   <= 1'b0;
        direct_data_valid_r <= 1'b0;
        direct_data_r       <= 32'h0;
        op_taken_r          <= 1'b0;
        op_ctrl_r           <= 1'b0;
        op_part_r           <= 1'b0;
        op_post_r           <= 1'b0;
        op_parts_r          <= 2'd0;
        op_part0_r          <= 32'h0;
        op_part1_r          <= 32'h0;
        wait_hold_r         <= 1'b0;
        wait_go_r           <= 1'b0;
        wait_slow_r         <= 5'd0;
    end else begin
        if (direct_release) begin
            direct_active       <= 1'b0;
            direct_issued_r     <= 1'b0;
            direct_data_valid_r <= 1'b0;
            op_taken_r          <= 1'b0;
            op_post_r           <= 1'b0;
        end

        op_part_r <= op_taken_r && operand_part;
        if (op_taken_r && operand_post)
            op_post_r <= 1'b1;
        if (op_part_r) begin
            if (op_parts_r == 2'd0)
                op_part0_r <= opr_r;
            else
                op_part1_r <= opr_r;
            op_parts_r <= op_parts_r + 2'd1;
        end

        if (dcmd_pending_r && br_accepted)
            dcmd_pending_r <= 1'b0;

        // The held WAIT word: done when the x87 is (go with nothing to
        // report, else run untaken for the routine to report), or when it
        // has worked 32 cycles in a row on something other than plain
        // arithmetic (run untaken: the routine waits, taking interrupts).
        if (wait_hold_r) begin
            if (wait_quiet) begin
                wait_hold_r <= 1'b0;
                wait_go_r   <= error_n;
            end else if (wait_slow_r == 5'd31) begin
                wait_hold_r <= 1'b0;
            end
            wait_slow_r <= queue_short ? 5'd0 : wait_slow_r + 5'd1;
        end
        // Its RNI is that one word's.
        if (store_exec)
            wait_go_r <= 1'b0;

        st_commit_r <= 1'b0;
        x87_quiet_r <= x87_quiet && !dcmd_pending_r;
        case (st_state)
            ST_IDLE: if (st_cmd) st_state <= br_accepted ? ST_READ : ST_CMD;
            ST_CMD:  if (br_accepted) st_state <= ST_READ;
            ST_READ: if (st_status_r) begin
                if (x87_quiet_r) begin
                    st_part0_r  <= {16'h0, st_control_r ? control_word : status_word};
                    st_commit_r <= 1'b1;
                    st_state    <= ST_DONE;
                end
            end else if (result_whole_valid) begin
                st_part0_r  <= result_whole_data[31:0];
                st_part1_r  <= result_whole_data[63:32];
                st_part2_r  <= result_whole_data[79:64];
                st_commit_r <= 1'b1;
                st_state    <= ST_DONE;
            end else if (result_valid) begin
                case (st_taken_r)
                    2'd0:    st_part0_r <= result_data;
                    2'd1:    st_part1_r <= result_data;
                    default: st_part2_r <= result_data[15:0];
                endcase
                st_taken_r <= st_taken_r + 2'd1;
                if (st_taken_r + 2'd1 == st_parts_r) begin
                    st_commit_r <= 1'b1;
                    st_state    <= ST_DONE;
                end
            end
            ST_DONE: if (store_exec) st_state <= ST_IDLE;
        endcase
        // A part's WR has taken OPR_R (its write data is in paging's
        // request): OPR_R takes the next part for the next WR.
        if (direct_st_r && next_part) begin
            st_part0_r  <= st_part1_r;
            st_part1_r  <= {16'h0, st_part2_r};
            st_commit_r <= 1'b1;
        end

        if (direct_launch) begin
            direct_active       <= ENABLE_X87 && esc_candidate && !l_reg &&
                                 !l_store && !l_status && !l_operand && direct_allowed &&
                                 queue_safe && !mask_load_pending;
            op_taken_r          <= ENABLE_X87 && esc_candidate && l_operand &&
                                 direct_allowed && queue_safe && !mask_load_pending;
            op_ctrl_r           <= l_fldcw;
            op_part_r           <= 1'b0;
            op_post_r           <= 1'b0;
            op_parts_r          <= 2'd0;
            // As the 386 routine: with every exception masked it may start
            // while the x87 is busy. A posted port write (BUSY# for it) is
            // waited out: the command enters the bridge behind it.
            // A status read is taken as the overlays are, with no unmasked
            // exception pending; it then waits for the x87 to finish.
            direct_st_r         <= ENABLE_X87 && esc_candidate && direct_allowed &&
                                 error_n && (l_status ||
                                             (l_store && (busy_n || queue_safe) &&
                                              !dcmd_pending_r));
            st_status_r         <= l_status;
            st_control_r        <= l_esc == 3'd1;
            st_state            <= (ENABLE_X87 && esc_candidate && direct_allowed &&
                                    error_n && l_status) ? ST_READ : ST_IDLE;
            st_parts_r          <= l_parts;
            st_taken_r          <= 2'd0;
            direct_reg_r        <= ENABLE_X87 && esc_candidate && l_regcmd &&
                                 direct_allowed && error_n && !dcmd_pending_r &&
                                 (((busy_n || queue_safe) && !posted_pending) ||
                                  (queue_safe && !mask_load_pending && queue_short));
            dcmd_pending_r      <= ENABLE_X87 && esc_candidate && l_regcmd &&
                                 direct_allowed && error_n && !dcmd_pending_r &&
                                 (((busy_n || queue_safe) && !posted_pending) ||
                                  (queue_safe && !mask_load_pending && queue_short));
            direct_issued_r     <= 1'b0;
            direct_crossing_r   <= 1'b0;
            direct_data_valid_r <= 1'b0;
            // WAIT: decided from what the x87 has in hand at the issue (a
            // status read waits on the same, registered a cycle later): go
            // when it is done with nothing pending, hold while it works;
            // otherwise the word runs untaken into the original routine.
            wait_go_r           <= wait_launch && wait_allowed &&
                                   (wait_nop || (wait_quiet && error_n));
            wait_hold_r         <= wait_launch && wait_allowed && !wait_nop && !wait_quiet;
            wait_slow_r         <= 5'd0;
        end

        if (direct_mem_req && mem_accepted) begin
            direct_issued_r   <= 1'b1;
            direct_crossing_r <= mem_addr_low != 2'b00;
        end

        if (direct_issued_r && !direct_crossing_r && mem_read_complete) begin
            direct_data_r       <= mem_rdata;
            direct_data_valid_r <= 1'b1;
        end else if (direct_issued_r && direct_crossing_r && !mem_servicing) begin
            direct_data_r       <= opr_r;
            direct_data_valid_r <= 1'b1;
        end

        if (cancel) begin
            direct_st_r         <= 1'b0;
            st_state            <= ST_IDLE;
            st_commit_r         <= 1'b0;
            direct_reg_r        <= 1'b0;
            dcmd_pending_r      <= 1'b0;
            direct_active       <= 1'b0;
            direct_issued_r     <= 1'b0;
            direct_data_valid_r <= 1'b0;
            op_taken_r          <= 1'b0;
            op_part_r           <= 1'b0;
            op_post_r           <= 1'b0;
            wait_hold_r         <= 1'b0;
            wait_go_r           <= 1'b0;
        end
    end
end

// synthesis translate_off
always_ff @(posedge clk) begin
    if (reset_n && direct_st_r && (st_state == ST_DONE) && result_valid)
        $error("x87 store result has more parts than its overlay writes");
    if (reset_n && dcmd_pending_r && (br_accepted != br_idle))
        $error("x87 register command acceptance differs from bridge idle");
end
// synthesis translate_on

`ifdef Z486_COSIM
// Co-simulation: the x87 has no command, transfer or result in flight, so its
// architectural state covers every ESC instruction the CPU has issued.
export "DPI-C" function cosim_x87_idle;
function int cosim_x87_idle();
    return int'(busy_n && !pereq && !dcmd_pending_r && (st_state == ST_IDLE));
endfunction
`endif

endmodule
