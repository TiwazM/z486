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
    input  logic [10:0] direct_fop,         // FOP of the executing instruction.
    input  logic [10:0] launch_fop,         // FOP of the issuing instruction (its x87 form).
    input  logic        operand_part,       // An X87_OPERAND word executed: OPR_R is next.
    input  logic        operand_post,       // An X87_POST word executed: OPR_R is the last part.
    input  logic        store_word,         // The store overlay's command/result word is current.
    input  logic        store_go,           // ...and its destination check completed (no memory stall).
    input  logic        store_exec,         // ...and executes.
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
wire        l_reg = launch_fop[7:6] == 2'b11;
wire  [2:0] l_esc = launch_fop[10:8];          // D8 + l_esc
wire  [2:0] l_op  = launch_fop[5:3];           // ModR/M.reg
// Stores: FST/FSTP m32 and FIST/FISTP m32 (D9, DB /2 /3).
wire        l_store = !l_reg && ((l_esc == 3'd1) || (l_esc == 3'd3)) &&
                      (l_op[2:1] == 2'b01);
// Loads whose RD words hand in the operand (x87-m16/m64/m80-load): DC-DF
// memory forms and DB /5; the m32 loads use direct_mem_req.
wire        l_operand = !l_reg && !l_store &&
                        (l_esc[2] || ((l_esc[1:0] == 2'b11) && (l_op == 3'd5)));

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
// executing instruction's first cycle. Taken only when the 386 routine would
// post it at once: CR0 allows x87 use, no unmasked exception is pending, the
// x87 is idle or may queue one command, and the bridge is free (no posted or
// direct command still waiting), so the bridge accepts it immediately.
logic        direct_reg_r;         // The issued instruction took the register path.
logic        dcmd_pending_r;       // ...and its command awaits bridge acceptance.
logic        posted_pending;       // The bridge holds a posted write.
logic        br_accepted, br_complete;
logic        br_read_complete;
logic [31:0] br_rdata;
// Stores: in the store word, after the destination check, post the FOP as a
// command, then take the result from the x87's transfer queue as it is
// produced (result_pop: no data-port read through the bridge). The store
// word holds until the result is in; OPR_R takes it the cycle the hold ends
// (ST_DONE), and the WR word after the store word writes it to memory.
typedef enum logic [1:0] {ST_IDLE, ST_CMD, ST_READ, ST_DONE} st_state_t;
st_state_t   st_state;
logic        direct_st_r;          // The issued instruction took the store path.
logic [31:0] st_part0_r;           // The result, for OPR_R.
logic        st_commit_r;          // ST_DONE's first cycle: OPR_R takes st_part0_r.
wire         st_cmd = direct_st_r && store_go && !cancel &&
                      (st_state == ST_IDLE || st_state == ST_CMD);
wire         result_pop = (st_state == ST_READ);
logic        result_valid;
logic [31:0] result_data;
wire         dcmd_sel = dcmd_pending_r || st_cmd;    // a command write (F8)
wire         own_cmd = dcmd_sel;
assign direct_taken = direct_active || direct_reg_r || direct_st_r || op_taken_r;
assign direct_reg_taken = direct_reg_r;
wire         direct_release = direct_valid && direct_ready;
assign load_commit = direct_release;

assign direct_mem_req = direct_active && !direct_issued_r && !direct_data_valid_r;
assign store_hold = direct_st_r && store_word && (st_state != ST_DONE);
assign store_opr_commit = st_commit_r;
assign store_record = st_commit_r;
assign store_opr_data = st_part0_r;
assign direct_stall = ((direct_active || op_post_r) && !direct_release) || store_hold;

generate
if (ENABLE_X87) begin : gen_x87
    wire        cmd_valid;
    wire [10:0] cmd_fop;
    wire        cmd_ready;
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
        .posted_pending(posted_pending),
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
        .busy_n(ctl_busy_n), .pereq(ctl_pereq), .error_n(error_n),
        .queue_safe(queue_safe), .mask_load_pending(mask_load_pending),
        .debug_state(debug_state)
    );
end else begin : gen_no_x87
    assign req_accepted = 1'b0;
    assign req_complete = 1'b0;
    assign result_valid = 1'b0;
    assign result_data = 32'h0;
    assign br_accepted = 1'b0;
    assign br_read_complete = 1'b0;
    assign br_rdata = 32'h0;
    assign br_complete = 1'b0;
    assign posted_pending = 1'b0;
    assign req_read_complete = 1'b0;
    assign req_rdata = 32'h0;
    assign busy_n = 1'b1;
    assign pereq = 1'b0;
    assign error_n = 1'b1;
    assign queue_safe = 1'b0;
    assign mask_load_pending = 1'b0;
    assign direct_ready = 1'b0;
    assign debug_state = 32'h8000_0000;
end
endgenerate

always_ff @(posedge clk) begin
    if (!reset_n) begin
        direct_st_r         <= 1'b0;
        st_state            <= ST_IDLE;
        st_part0_r          <= 32'h0;
        st_commit_r         <= 1'b0;
        direct_reg_r        <= 1'b0;
        dcmd_pending_r      <= 1'b0;
        direct_active       <= 1'b0;
        direct_issued_r     <= 1'b0;
        direct_crossing_r   <= 1'b0;
        direct_data_valid_r <= 1'b0;
        direct_data_r       <= 32'h0;
        op_taken_r          <= 1'b0;
        op_part_r           <= 1'b0;
        op_post_r           <= 1'b0;
        op_parts_r          <= 2'd0;
        op_part0_r          <= 32'h0;
        op_part1_r          <= 32'h0;
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

        st_commit_r <= 1'b0;
        case (st_state)
            ST_IDLE: if (st_cmd) st_state <= br_accepted ? ST_READ : ST_CMD;
            ST_CMD:  if (br_accepted) st_state <= ST_READ;
            ST_READ: if (result_valid) begin
                st_part0_r  <= result_data;
                st_commit_r <= 1'b1;
                st_state    <= ST_DONE;
            end
            ST_DONE: if (store_exec) st_state <= ST_IDLE;
        endcase

        if (direct_launch) begin
            direct_active       <= ENABLE_X87 && direct_candidate && !l_reg &&
                                 !l_store && !l_operand && direct_allowed &&
                                 queue_safe && !mask_load_pending;
            op_taken_r          <= ENABLE_X87 && direct_candidate && l_operand &&
                                 direct_allowed && queue_safe && !mask_load_pending;
            op_part_r           <= 1'b0;
            op_post_r           <= 1'b0;
            op_parts_r          <= 2'd0;
            // As the 386 routine: with every exception masked it may start
            // while the x87 is busy. A posted port write (BUSY# for it) is
            // waited out: the command enters the bridge behind it.
            direct_st_r         <= ENABLE_X87 && direct_candidate && l_store &&
                                 direct_allowed && error_n && (busy_n || queue_safe) &&
                                 !dcmd_pending_r;
            st_state            <= ST_IDLE;
            direct_reg_r        <= ENABLE_X87 && direct_candidate && l_reg &&
                                 direct_allowed && error_n && (busy_n || queue_safe) &&
                                 !posted_pending && !dcmd_pending_r;
            dcmd_pending_r      <= ENABLE_X87 && direct_candidate && l_reg &&
                                 direct_allowed && error_n && (busy_n || queue_safe) &&
                                 !posted_pending && !dcmd_pending_r;
            direct_issued_r     <= 1'b0;
            direct_crossing_r   <= 1'b0;
            direct_data_valid_r <= 1'b0;
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
        end
    end
end

`ifdef Z486_COSIM
// Co-simulation: the x87 has no command, transfer or result in flight, so its
// architectural state covers every ESC instruction the CPU has issued.
export "DPI-C" function cosim_x87_idle;
function int cosim_x87_idle();
    return int'(busy_n && !pereq && !dcmd_pending_r && (st_state == ST_IDLE));
endfunction
`endif

endmodule
