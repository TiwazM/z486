//
// x87 Control
// Command decode, microsequencing, status, CPU transfers and the register stack
//
module x87_control
    import x87_pkg::*, x87_ucode_pkg::*;
(
    input  logic        clk,                    // Core/system clock.
    input  logic        reset,                  // Synchronous active-high reset.

    input  logic        cmd_valid,              // CPU presents an x87 command.
    input  logic [10:0] cmd_fop,                // Encoded ESC opcode and ModR/M.
    output logic        cmd_ready,              // Core can accept cmd_fop now (perhaps
                                                // to hold it behind a running operation).
    output logic        quiet,                  // Every accepted command has completed,
                                                // status word included; none is held.

    input  logic        direct_valid,           // Direct path posts a fault-checked memory operand.
    input  logic [10:0] direct_fop,             // Memory-form FOP for direct_data.
    input  logic [79:0] direct_data,            // Operand, low part first, one dword per fragment.
    input  logic  [1:0] direct_words,           // Fragments: 1 (m16/m32), 2 (m64), 3 (m80).
    input  logic        direct_tail16,          // The last fragment holds two bytes.
    output logic        direct_ready,           // One-entry direct command queue is available.

    input  logic        word_in_valid,          // Operand-transfer word is valid.
    input  logic  [3:0] word_in_be,             // Valid bytes in word_in_data.
    input  logic [31:0] word_in_data,           // CPU-to-x87 transfer data.
    output logic        word_in_ready,          // Core can accept a transfer word.

    input  logic        read_req_valid,         // CPU requests status or result data.
    input  logic        read_req_data_port,     // 1=result data; 0=status word.
    input  logic  [3:0] read_req_be,            // Requested result-data byte lanes.
    output logic        read_req_ready,         // Core can accept the read request.
    output logic        read_resp_valid,        // read_resp_data is valid.
    output logic [31:0] read_resp_data,         // x87-to-CPU status/result data.

    input  logic        result_pop,             // A direct store takes the head result fragment.
    output logic        result_valid,           // A store result fragment heads the queue.
    output logic [31:0] result_data,            // ...as the data port would return it.
    output logic [15:0] status_out,             // Status word, for a direct FNSTSW.
    output logic [15:0] control_out,            // Control word, for a direct FNSTCW.

    output logic        busy_n,                 // Active-low arithmetic busy status.
    output logic        pereq,                  // CPU operand-transfer/completion request.
    output logic        error_n,                // Active-low unmasked-error status.
    output logic        queue_safe,             // Masked exceptions permit one queued command.
    output logic        mask_load_pending,      // FLDCW, FLDENV or FRSTOR has not loaded its masks.
    output logic        queue_short,            // A plain operation runs and at most one short
                                                // command is held: the bridge frees soon.

    // Low-rate hardware diagnostics; sampled only by the MiSTer watchdog.
    output logic [31:0] debug_state             // Packed command/executor progress.
);

typedef enum logic [3:0] {
    RX_NONE,
    RX_CONTROL,
    RX_M32,
    RX_M64,
    RX_M80,
    RX_I16,
    RX_I32,
    RX_I64,
    RX_ENV,
    RX_STATE,
    RX_BCD
} rx_kind_t;

typedef enum logic [1:0] {
    TX_NONE,
    TX_VALUE,
    TX_ENV,
    TX_STATE
} tx_kind_t;

typedef enum logic [1:0] {
    EXEC_NONE,
    EXEC_LOAD,
    EXEC_STORE,
    EXEC_MATH
} convert_owner_t;

logic [2:0] top;                       // Physical index of architectural ST(0).
logic [15:0] control_word;             // Exception masks, PC, RC, and infinity control.
logic [15:0] status_flags;             // Sticky exceptions, condition codes, ES, and B.
logic [10:0] last_fop;                 // Most recently accepted architectural FOP.
logic [15:0] tag_word;                 // Two architectural tag bits per physical stack row.
// Tag-word updates are collected as requests while the clocked block runs and
// applied once at its end: a whole-word load, port A and port B row writes
// (alongside the stack RAM ports), then rows popped from TOP. One shared
// update replaces a row decoder and value mux per call site.
logic        tag_full_we;
logic [15:0] tag_full_value;
logic        tag_a_we;
logic  [2:0] tag_a_index;
logic  [1:0] tag_a_value;
logic        tag_b_we;
logic  [2:0] tag_b_index;
logic  [1:0] tag_b_value;
logic  [1:0] tag_pop_count;
logic        command_pending;          // Command ROM and stack RAM return next cycle.
logic [10:0] command_fop;              // FOP retained through dispatch and status policy.
x87_command_decode_t command_decode;   // Synchronous generated command descriptor.
x87_command_decode_t command_decode_rom;
x87_exec_op_t        v2_exec_op_port;  // Executor operation as presented to the port.
logic        direct_pending;           // One direct FOP/operand pair waiting for dispatch.
logic [10:0] direct_fop_r;
logic [79:0] direct_data_r;
logic  [1:0] direct_words_r;
logic        direct_tail16_r;
logic        masked_cw_load_r;         // The command is a direct FLDCW of a control
                                       // word that keeps every exception masked.
logic  [1:0] direct_tail_words;        // Fragments still to enter the transfer queue.
logic [47:0] direct_tail_data;         // ...low fragment first.

logic [2:0] stack_addr_a;              // Retained physical stack address for port A.
logic [2:0] stack_addr_b;              // Retained physical stack address for port B.
logic [2:0] stack_port_addr_a;
logic [2:0] stack_port_addr_b;
logic       stack_write_a;
logic       stack_write_b;
logic [79:0] stack_write_data_a;
logic [79:0] stack_write_data_b;
logic [79:0] stack_read_raw_a;         // Stack RAM outputs, with a held command's
logic [79:0] stack_read_raw_b;         // forwarded row (below).
logic [79:0] stack_ram_q_a;
logic [79:0] stack_ram_q_b;
// A command may be accepted while a plain executor operation (register or
// memory arithmetic, a compare, FSQRT, FRNDINT, FSCALE) runs, with every exception
// masked. It reads its stack rows at acceptance, relative to TOP after that
// operation's pops, and is held (command_pending, command_held) until the
// operation has retired; then it dispatches in the cycle its result is written
// to the RAM, taking that row from the write (it read the row before).
logic        exec_plain;               // A plain executor operation runs (cleared at done).
logic        command_held;             // The pending command was accepted behind it.
logic  [2:0] command_row_a;            // Rows the pending command read at acceptance.
logic  [2:0] command_row_b;
logic        command_rows_fresh;       // The RAM outputs are still those reads: the ports
                                       // kept those rows last cycle and wrote nothing.
x87_reg_t    stack_read_data_a;
x87_reg_t    stack_read_data_b;

rx_kind_t rx_kind;                     // Active CPU-to-x87 transfer format.
logic [4:0] rx_index;                  // Environment/state stream position.
logic [79:0] rx_payload;               // Byte-compacted current numeric operand.
logic [3:0]  rx_byte_count;            // Valid bytes accumulated in rx_payload.
logic [159:0] rx_state_shift;           // Two raw m80 values during FRSTOR.

logic [1:0] tx_count;
logic [3:0] tx_last_be;
tx_kind_t tx_kind;                     // Active x87-to-CPU transfer format.
logic [4:0] tx_index;
logic [159:0] tx_state_shift;           // Value/state fragments awaiting FIFO production.
logic         tx_generation_done;
logic [2:0]   tx_byte_offset;
logic         transfer_push_valid;
logic [35:0]  transfer_push_data;
logic         transfer_push_ready;
logic         transfer_pop_valid;
logic [35:0]  transfer_pop_data;
logic         transfer_pop_ready;
logic [1:0]   transfer_count;
logic         tx_produce_valid;
logic [35:0]  tx_produce_data;
logic         tx_produce_fire;
logic         tx_consume_fire;
logic         status_read_pending;
logic         command_complete_pulse;  // Extends terminal PEREQ visibility.
logic [1:0]   pereq_release_hold;       // Covers later 80386 CORWAIT sampling slots.
logic         push_pending;            // Serialized architectural stack push.
logic [79:0]  push_pending_raw;
logic [1:0]   push_pending_tag;
x87_reg_t     push_pending_value;
logic         memory_math_pending;     // FOP accepted; memory operand not converted yet.
logic         memory_math_mul;
logic         memory_math_div;
logic         memory_math_compare;
logic         memory_math_subtract;
logic         memory_math_reverse;
logic         memory_math_pop;
x87_reg_t     memory_math_st0;         // ST0 as the command dispatched (forwarded).
logic         store_pending;           // ST0 conversion/transfer setup is pending.
logic         store_integer;
logic [1:0]   store_width;
logic         store_pop;
logic         store_source_empty;
x87_reg_t     store_source;
logic [79:0]  store_source_raw;
logic         store_bcd;
logic         bcd_convert_pending;
logic  [6:0]  bcd_shift_count;
logic         bcd_sign;
logic         bcd_load_pending;          // FBLD: BCD-to-binary steps in tx_state_shift.
logic         bcd_load_negate;           // FBLD: the loaded value takes the BCD sign.
logic         pop_pending;
logic         result_write_pending;    // Serialized architectural stack replacement.
logic [2:0]   result_write_index;
logic [79:0]  result_write_raw;
logic                   v2_exec_pending; // Provisional numeric operation awaits start.
x87_exec_op_t           v2_exec_op;      // Operation selected for x87_executor.
convert_owner_t         v2_exec_owner;   // Load, store, or arithmetic retirement policy.
logic             [1:0] v2_exec_size;
logic            [63:0] v2_exec_transfer;
logic                   v2_exec_busy;     // Numeric microsequencer is active.
logic                   v2_exec_done;     // Registered terminal pulse from executor.
logic             [2:0] v2_exec_commit;   // Provisional action; control applies exceptions.
x87_reg_t               v2_exec_result;
logic            [63:0] v2_exec_transfer_out;
logic                   v2_exec_invalid;
logic                   v2_exec_inexact;
logic                   v2_exec_divide_by_zero;
logic                   v2_exec_overflow;
logic                   v2_exec_underflow;
logic                   v2_exec_denormal_operand;
logic                   v2_exec_range_incomplete;
logic                   v2_exec_rounded_up;
logic                   v2_compare_unordered;
logic                   v2_compare_less;
logic                   v2_compare_equal;

logic         arith_compare;             // Current math result updates C3/C2/C0 only.
logic         arith_quiet_compare;
logic         arith_write_result;
logic [1:0]   arith_pop_count;
logic [2:0]   arith_dest_index;
x87_reg_t     arith_operand_a;
x87_reg_t     arith_operand_b;
logic         trans_cosine;
logic         trans_atan2;
logic   [1:0] trans_logexp;              // X87_TRANS_F2XM1/FYL2X/FYL2XP1, else 0.
logic         fptan_trans_pending;       // CORDIC tangent pair has not returned.
logic         fptan_div_pending;         // Shared divide is producing tan = sin/cos.
logic         fptan_push_after_result;   // Tangent write precedes architectural push 1.0.
logic         fptan_sincos;              // FSINCOS: push cos(x) instead of 1.0, no divide.

// Transcendental programs (build_x87_poly.py): FSIN/FCOS/FSINCOS/FPTAN/FPATAN,
// F2XM1, FYL2X and FYL2XP1 of finite, non-special operands run as sequences
// of the executor's own operations; the executor's TRANS operation only
// classifies (special results commit directly, the rest finish with no commit
// and start a program).
`include "x87_poly_rom.svh"
typedef enum logic [3:0] {
    POLY_IDLE,
    POLY_LOAD_B,       // second input into the scratch
    POLY_DECODE,       // program word read
    POLY_FETCH_A,      // predicate; operand A read
    POLY_FETCH_B,      // operand B read
    POLY_ISSUE,        // start the executor operation (or AUX, XTR)
    POLY_WAIT,         // executor result into the scratch / predicates
    POLY_XTR_NORM,     // XTR: normalize (a denormal, one bit per cycle), m >= sqrt(2)
    POLY_XTR_M,        // XTR: m into the scratch; k's offset sum
    POLY_XTR_K         // XTR: 2^27 + 2^15 + k into the scratch; k == 0
} poly_phase_t;
poly_phase_t     poly_phase;
logic [7:0]      poly_pc;
x87_poly_word_t  poly_word;
// While an operation runs (POLY_WAIT), the next word is fetched: read when
// the operation issues, its predicate checked (a word that does not run is
// skipped) and A and B read through the scratch and constant ports the wait
// leaves idle, both at once when one is a constant and the other is not. At
// the operation's done the next one issues at once; an operand that reads
// the slot being written takes the executor's result. Not after CMP or RND
// (they change the predicates and the reduction id) or the final word, and
// not for AUX or XTR (control-only words): those, and a fetch not complete
// at done, continue as before at pf_pc.
typedef enum logic [2:0] {
    PF_IDLE,           // no fetch: continue at poly_pc + 1
    PF_PRED,           // pf_word in: predicate; reads (A, and B on the other port)
    PF_CAP1,           // A and B in
    PF_CAPA,           // A in; B's read
    PF_CAPB,           // B in
    PF_READY,          // issue at done
    PF_STOP            // AUX or XTR at pf_pc: continue there as before
} poly_fetch_t;
poly_fetch_t     pf_state;
logic [7:0]      pf_pc;
x87_poly_word_t  pf_word;
x87_reg_t        pf_opa;
x87_reg_t        pf_opb;
logic [1:0]      poly_quadrant;            // n mod 4 of the pi/2 reduction
logic            poly_swap;                // atan2: |y| > |x|
logic [1:0]      poly_id;                  // atan2: reduction table index
logic            poly_xneg, poly_yneg;
logic            poly_cosine;
logic            poly_final;               // the executor runs the last operation
logic            poly_denormal;            // the classified operand was denormal
logic            poly_k0;                  // XTR: k == 0 (set at the start)
logic            poly_exact;               // XTR: m == 1; PE from the final rounding only
// XTR splits a positive A into 2^k * m, sqrt(1/2) <= m < sqrt(2): m into
// dest, 2^27 + 2^15 + k (k's 16-bit offset binary at 2^0) into the slot
// b_index names. A denormal (or unnormal) shifts left one bit per cycle.
logic [52:0]     xtr_sig;
logic            xtr_guard, xtr_round, xtr_sticky;
logic signed [16:0] xtr_exp;               // A's exponent (biased; 1 for a denormal)
logic            xtr_ge;                   // sig >= sqrt(2): m = sig/2, k + 1
logic [15:0]     xtr_u;                    // k + 2^15
x87_reg_t        poly_aux;                 // cos(x) for FPTAN and FSINCOS
x87_reg_t        poly_opa;
x87_reg_t        poly_scratch [0:15];      // temporaries (x87_reg_t, no conversion)
x87_reg_t        poly_scratch_q;
x87_reg_t        poly_const_q;
logic            poly_we;
logic [3:0]      poly_waddr;
x87_reg_t        poly_wdata;
logic [3:0]      poly_saddr;
logic [6:0]      poly_caddr;
x87_reg_t     fptan_push_value;          // Value pushed after the ST0 write (1.0 or cos).
typedef enum logic [2:0] {
    FPREM_IDLE,
    FPREM_DIVIDE,
    FPREM_ROUND,
    FPREM_MULTIPLY,
    FPREM_SUBTRACT
} fprem_phase_t;
fprem_phase_t fprem_phase;
x87_reg_t     fprem_dividend;
x87_reg_t     fprem_divisor;
logic [2:0]   fprem_quotient_bits;
logic         fprem_nearest;             // FPREM1: round the quotient to nearest.
logic         fxtract_exp_pending;       // FXTRACT: the FILD of the exponent writes here.
logic [2:0]   fxtract_exp_index;
logic   [7:0] v2_exec_uaddr;

wire [15:0] status_word = {status_flags[15:14], top, status_flags[10:0]};
wire [2:0] st0_index = top;
wire [2:0] cmd_st_index = top + command_fop[2:0];
assign debug_state = {
    busy_n, cmd_ready, command_pending, v2_exec_busy,
    v2_exec_done, v2_exec_pending, v2_exec_owner,
    v2_exec_uaddr, last_fop, rx_kind, (tx_kind != TX_NONE)
};
// A held command dispatches while the stack RAM writes its predecessor's
// result (port A); a row it reads from that write comes from the write data.
wire forward_a = command_held && stack_write_a && (stack_addr_a == command_row_a);
wire forward_b = command_held && stack_write_a && (stack_addr_a == command_row_b);
assign stack_read_raw_a = forward_a ? stack_write_data_a : stack_ram_q_a;
assign stack_read_raw_b = forward_b ? stack_write_data_a : stack_ram_q_b;
assign stack_read_data_a = x87_from_m80(stack_read_raw_a);
assign stack_read_data_b = x87_from_m80(stack_read_raw_b);
assign push_pending_value = x87_from_m80(push_pending_raw);
wire command_is_arithmetic = command_decode.arithmetic;
wire v2_exec_math_done = v2_exec_done &&
                            (v2_exec_owner == EXEC_MATH);
wire trans_exec_done = v2_exec_math_done &&
                       (v2_exec_op == X87_ARITH_TRANS);
// The executor classified a finite operand: a program computes the result.
wire trans_compute = trans_exec_done && (v2_exec_commit == X87_COMMIT_NONE);
wire poly_active = (poly_phase != POLY_IDLE);
wire poly_internal = poly_active && !poly_final;
wire poly_final_done = v2_exec_math_done && poly_final;
wire trans_done = (trans_exec_done && !trans_compute) || poly_final_done;
wire trans_inexact = v2_exec_inexact || (poly_final_done && !poly_exact);
wire trans_denormal_operand = v2_exec_denormal_operand ||
                              (poly_final_done && poly_denormal);
x87_reg_t trans_aux;
assign trans_aux = poly_final_done ? poly_aux : x87_one();
wire math_done = v2_exec_math_done && (fprem_phase == FPREM_IDLE) &&
                 !(trans_done && fptan_trans_pending) &&
                 !trans_compute && !poly_internal;
wire math_invalid = v2_exec_invalid;
wire math_inexact = trans_inexact;
wire [74:0] math_result = v2_exec_result;
wire math_divide_by_zero = v2_exec_math_done &&
                           v2_exec_divide_by_zero;
wire math_overflow = v2_exec_math_done && v2_exec_overflow;
wire math_underflow = v2_exec_math_done && v2_exec_underflow;
wire math_denormal_operand = v2_exec_math_done &&
                              trans_denormal_operand;
wire math_range_incomplete = trans_done &&
                             v2_exec_range_incomplete;
// The math destination is a row the same instruction pops.
wire arith_pop_row_written =
    ((arith_pop_count != 2'd0) && (arith_dest_index == top)) ||
    ((arith_pop_count == 2'd2) && (arith_dest_index == top + 3'd1));
wire math_unmasked_exception =
    (math_invalid && !control_word[0]) ||
    (math_denormal_operand && !control_word[1]) ||
    (math_divide_by_zero && !control_word[2]) ||
    (math_overflow && !control_word[3]) ||
    (math_underflow && !control_word[4]) ||
    (math_inexact && !control_word[5]);

function automatic logic fop_reads_status(input logic [10:0] fop);
    return (fop == 11'h7e0) ||
           ((fop[10:8] == 3'd5) && (fop[7:6] != 2'b11) &&
            (fop[5:3] == 3'b111));
endfunction

// FLDCW, FLDENV and FRSTOR load the exception masks.
function automatic logic fop_loads_masks(input logic [10:0] fop);
    return (fop[7:6] != 2'b11) &&
           (((fop[10:8] == 3'd1) && ((fop[5:3] == 3'd4) || (fop[5:3] == 3'd5))) ||
            ((fop[10:8] == 3'd5) && (fop[5:3] == 3'd4)));
endfunction

function automatic logic [2:0] transfer_byte_count(input logic [3:0] be);
    return {2'b00, be[0]} + {2'b00, be[1]} +
           {2'b00, be[2]} + {2'b00, be[3]};
endfunction

function automatic logic [79:0] place_transfer_word(
    input logic [79:0] payload,
    input logic [3:0]  byte_offset,
    input logic [31:0] data,
    input logic [3:0]  be
);
    logic [79:0] merged;
    logic [2:0]  slot;
    integer half;
    begin
        merged = payload;
        for (half = 0; half < 2; half = half + 1) begin
            slot = byte_offset[3:1] + half[2:0];
            if (be[half*2] && (slot < 3'd5))
                merged[slot*16 +: 16] = data[half*16 +: 16];
        end
        return merged;
    end
endfunction

function automatic logic [31:0] select_transfer_bytes(
    input logic [31:0] data,
    input logic [2:0]  byte_offset,
    input logic [3:0]  be
);
    logic [31:0] selected;
    integer lane;
    integer source;
    begin
        selected = 32'h0;
        source = byte_offset;
        for (lane = 0; lane < 4; lane = lane + 1) begin
            if (be[lane] && (source < 4)) begin
                selected[lane*8 +: 8] = data[source*8 +: 8];
                source = source + 1;
            end
        end
        return selected;
    end
endfunction

// Memory FOPs encode the complete ModR/M byte, but addressing-mode and r/m
// bits are not part of the x87 operation. Preserve register forms verbatim
// and canonicalize memory forms to opcode plus ModR/M.reg for command decode.
function automatic logic [10:0] fop_decode_key(input logic [10:0] fop);
    if (fop[7:6] == 2'b11)
        return fop;
    return {fop[10:8], 2'b00, fop[5:3], 3'b000};
endfunction

function automatic logic [2:0] x87_integer_low3(input x87_reg_t value);
    logic signed [16:0] unbiased;
    logic [63:0] magnitude;
    begin
        unbiased = $signed({2'b00, value.exp}) - 17'sd16383;
        if ((value.class_id == X87_ZERO) || (unbiased < 0))
            return 3'b000;
        if (unbiased > 54)
            return 3'b000;
        if (unbiased >= 52)
            magnitude = {11'b0, value.sig} << (unbiased - 52);
        else
            magnitude = {11'b0, value.sig} >> (52 - unbiased);
        return magnitude[2:0];
    end
endfunction

function automatic logic m32_is_normal(input logic [31:0] raw);
    return (raw[30:23] != 8'h00) && (raw[30:23] != 8'hff);
endfunction

function automatic logic m64_is_normal(input logic [63:0] raw);
    return (raw[62:52] != 11'h000) && (raw[62:52] != 11'h7ff);
endfunction

// A normal m32 or m64 operand as x87_from_m80 reads its extended form.
function automatic x87_reg_t normal_m32_value(input logic [31:0] raw);
    x87_reg_t value;
    value = '0;
    value.sign = raw[31];
    value.exp = {7'h0, raw[30:23]} + 15'd16256;
    value.sig = {1'b1, raw[22:0], 29'h0};
    value.class_id = X87_NORMAL;
    return value;
endfunction

function automatic x87_reg_t normal_m64_value(input logic [63:0] raw);
    x87_reg_t value;
    value = '0;
    value.sign = raw[63];
    value.exp = {4'h0, raw[62:52]} + 15'd15360;
    value.sig = {1'b1, raw[51:0]};
    value.class_id = X87_NORMAL;
    return value;
endfunction

// A normal m64 extends exactly, as the executor's FLD_M64 conversion does it.
function automatic logic [79:0] normal_m64_to_m80(input logic [63:0] raw);
    logic [14:0] extended_exp;
    begin
        extended_exp = {4'h0, raw[62:52]} + 15'd15360;
        return {raw[63], extended_exp, 1'b1, raw[51:0], 11'h0};
    end
endfunction

function automatic logic [79:0] normal_m32_to_m80(input logic [31:0] raw);
    logic [14:0] extended_exp;
    begin
        extended_exp = {7'h0, raw[30:23]} + 15'd16256;
        return {raw[31], extended_exp, 1'b1, raw[22:0], 40'h0};
    end
endfunction


function automatic logic [15:0] architectural_tag_word();
    return tag_word;
endfunction

function automatic logic stack_empty(input logic [2:0] index);
    return tag_word[index*2 +: 2] == 2'b11;
endfunction

function automatic logic [1:0] stack_tag_from_m80(input logic [79:0] value);
    logic [14:0] exponent;
    logic [63:0] significand;
    begin
        exponent = value[78:64];
        significand = value[63:0];
        if ((exponent == 0) && (significand == 0))
            return 2'b01; // Zero.
        if ((exponent != 0) && (exponent != 15'h7fff) && significand[63])
            return 2'b00; // Valid finite value.
        return 2'b10;     // Denormal, infinity, NaN, or unsupported encoding.
    end
endfunction

function automatic logic [159:0] pack_state_pair(
    input logic [79:0] even_value,
    input logic [79:0] odd_value,
    input logic [2:0] even_index
);
    logic [79:0] saved_even;
    logic [79:0] saved_odd;
    begin
        saved_even = stack_empty(even_index)
                   ? x87_to_m80(x87_empty()) : even_value;
        saved_odd = stack_empty(even_index + 3'd1)
                  ? x87_to_m80(x87_empty()) : odd_value;
        return {saved_odd, saved_even};
    end
endfunction

task automatic set_tag_a(input logic [2:0] index, input logic [1:0] value);
    begin
        // synthesis translate_off
        if (tag_a_we) $error("x87 tag port A written twice in one cycle");
        if ((tag_pop_count != 0) && ((index == top) ||
            ((tag_pop_count == 2) && (index == top + 3'd1))))
            $error("x87 tag port A write after a pop of the same row");
        // synthesis translate_on
        tag_a_we = 1'b1;
        tag_a_index = index;
        tag_a_value = value;
    end
endtask

task automatic set_tag_b(input logic [2:0] index, input logic [1:0] value);
    begin
        // synthesis translate_off
        if (tag_b_we) $error("x87 tag port B written twice in one cycle");
        // synthesis translate_on
        tag_b_we = 1'b1;
        tag_b_index = index;
        tag_b_value = value;
    end
endtask

task automatic set_tag_word(input logic [15:0] value);
    begin
        tag_full_we = 1'b1;
        tag_full_value = value;
    end
endtask

task automatic pop_tags(input logic [1:0] count);
    begin
        tag_pop_count = count;
    end
endtask

task automatic clear_stack;
    begin
        set_tag_word(16'hffff);
    end
endtask

task automatic write_stack(input logic [2:0] index, input x87_reg_t value);
    begin
        stack_addr_a <= index;
        stack_write_data_a <= x87_to_m80(value);
        stack_write_a <= 1'b1;
        set_tag_a(index, x87_tag(value));
    end
endtask

task automatic write_stack_raw(
    input logic [2:0] index,
    input logic [79:0] value
);
    begin
        stack_addr_a <= index;
        stack_write_data_a <= value;
        stack_write_a <= 1'b1;
        set_tag_a(index, stack_tag_from_m80(value));
    end
endtask

task automatic write_stack_raw_tagged(
    input logic [2:0] index,
    input logic [79:0] value,
    input logic [1:0] value_tag
);
    begin
        stack_addr_a <= index;
        stack_write_data_a <= value;
        stack_write_a <= 1'b1;
        set_tag_a(index, value_tag);
    end
endtask

task automatic raise_stack_fault(input logic overflow);
    begin
        status_flags[0] <= 1'b1; // IE
        status_flags[6] <= 1'b1; // SF
        status_flags[9] <= overflow; // C1 distinguishes overflow/underflow
        if (!control_word[0]) begin
            status_flags[7] <= 1'b1; // ES: unmasked exception summary
            status_flags[15] <= 1'b1; // B
        end
    end
endtask

task automatic raise_invalid;
    begin
        status_flags[0] <= 1'b1;
        if (!control_word[0]) begin
            status_flags[7] <= 1'b1;
            status_flags[15] <= 1'b1;
        end
    end
endtask

task automatic push_raw_tagged(
    input logic [79:0] value,
    input logic [1:0] value_tag
);
    logic [2:0] new_top;
    begin
        new_top = top - 3'd1;
        if (stack_empty(new_top)) begin
            top <= new_top;
            write_stack_raw_tagged(new_top, value, value_tag);
            status_flags[9] <= 1'b0;
        end else begin
            raise_stack_fault(1'b1);
            if (control_word[0]) begin
                top <= new_top;
                write_stack(new_top, x87_indefinite());
            end
        end
    end
endtask

task automatic push_value(input x87_reg_t value);
    begin
        push_raw_tagged(x87_to_m80(value), x87_tag(value));
    end
endtask

task automatic schedule_push(input x87_reg_t value);
    begin
        push_pending_raw <= x87_to_m80(value);
        push_pending_tag <= x87_tag(value);
        push_pending <= 1'b1;
        command_complete_pulse <= 1'b0;
    end
endtask

task automatic push_const_raw(input logic [79:0] value);
    begin
        push_raw_tagged(value, stack_tag_from_m80(value));
    end
endtask

task automatic pop_value;
    begin
        pop_tags(2'd1);
        top <= top + 3'd1;
    end
endtask

// Memory arithmetic: the operand meets ST0 as it was when the command
// dispatched (nothing else writes the stack meanwhile).
task automatic launch_memory_math(input x87_reg_t operand);
    begin
        memory_math_pending <= 1'b0;
        if (stack_empty(st0_index)) begin
            raise_stack_fault(1'b0);
            if (memory_math_compare) begin
                status_flags[14] <= 1'b1;
                status_flags[10] <= 1'b1;
                status_flags[8] <= 1'b1;
            end
            if (control_word[0]) begin
                if (!memory_math_compare)
                    write_stack(st0_index, x87_indefinite());
                if (memory_math_pop)
                    pop_value();
            end
        end else begin
            command_complete_pulse <= 1'b0;
            exec_plain <= 1'b1;
            arith_compare <= memory_math_compare;
            arith_quiet_compare <= 1'b0;
            arith_write_result <= !memory_math_compare;
            arith_pop_count <= {1'b0, memory_math_pop};
            arith_dest_index <= st0_index;
            arith_operand_a <= memory_math_reverse ? operand : memory_math_st0;
            arith_operand_b <= memory_math_reverse ? memory_math_st0 : operand;
            v2_exec_op <= memory_math_div
                ? X87_ARITH_DIV
                : memory_math_mul
                ? X87_ARITH_MUL
                : memory_math_compare
                ? X87_ARITH_COMPARE
                : memory_math_subtract ? X87_ARITH_SUB
                                       : X87_ARITH_ADD;
            v2_exec_owner <= EXEC_MATH;
            v2_exec_size <= 2'd0;
            v2_exec_transfer <= 64'h0;
            v2_exec_pending <= 1'b1;
        end
    end
endtask

task automatic start_store(
    input logic integer_store,
    input logic [1:0] width,
    input logic pop
);
    logic source_empty;
    begin
        source_empty = stack_empty(st0_index);
        store_pending <= 1'b1;
        store_bcd <= 1'b0;
        store_integer <= integer_store;
        store_width <= width;
        store_pop <= pop;
        store_source_empty <= source_empty;
        // Store commands request CPU reads only after converted words enter
        // the output FIFO.
        command_complete_pulse <= 1'b0;
        if (source_empty) begin
            raise_stack_fault(1'b0);
            store_source <= x87_indefinite();
            store_source_raw <= x87_to_m80(x87_indefinite());
        end else begin
            store_source <= stack_read_data_a;
            store_source_raw <= stack_read_raw_a;
            status_flags[9] <= 1'b0;
        end
    end
endtask

task automatic start_bcd_store;
    logic source_empty;
    begin
        source_empty = stack_empty(st0_index);
        store_pending <= 1'b1;
        store_bcd <= 1'b1;
        store_integer <= 1'b1;
        store_width <= 2'd2;
        store_pop <= 1'b1;
        store_source_empty <= source_empty;
        command_complete_pulse <= 1'b0;
        if (source_empty) begin
            raise_stack_fault(1'b0);
            store_source <= x87_indefinite();
            store_source_raw <= x87_to_m80(x87_indefinite());
        end else begin
            store_source <= stack_read_data_a;
            store_source_raw <= stack_read_raw_a;
            status_flags[9] <= 1'b0;
        end
    end
endtask

// A loaded control word keeps the reserved bits as a 387/486 does: 15:13
// and 7 read as 0, bit 6 as 1 (FNINIT's 037Fh).
function automatic logic [15:0] x87_load_control_word(input logic [15:0] value);
    return (value & ~16'he0c0) | 16'h0040;
endfunction

task automatic accept_environment_word(
    input logic [2:0] word_index,
    input logic [15:0] value
);
    begin
        case (word_index)
            3'd0: control_word <= x87_load_control_word(value);
            3'd1: begin
                status_flags <= value;
                top <= value[13:11];
            end
            3'd2: set_tag_word(value);
            default: ; // Instruction and operand pointers live in the 80386.
        endcase
    end
endtask

task automatic commit_state_pair(
    input logic [1:0] pair_index,
    input logic [31:0] final_word
);
    logic [159:0] completed_pair;
    logic [2:0] even_index;
    logic [2:0] odd_index;
    begin
        completed_pair = {final_word, rx_state_shift[127:0]};
        even_index = top + {pair_index, 1'b0};
        odd_index = even_index + 3'd1;
        stack_addr_a <= even_index;
        stack_write_data_a <= completed_pair[79:0];
        stack_write_a <= 1'b1;
        stack_addr_b <= odd_index;
        stack_write_data_b <= completed_pair[159:80];
        stack_write_b <= 1'b1;
    end
endtask

// RPTI may replay a memory command after the x87 has accepted it but before
// the CPU transfers the first operand word. Re-arming that exact empty receive
// transaction is idempotent; unrelated commands remain blocked.
wire core_cmd_direct = direct_pending;
wire core_cmd_valid = core_cmd_direct || cmd_valid;
wire [10:0] core_cmd_fop = core_cmd_direct ? direct_fop_r : cmd_fop;
wire restartable_rx_command = !core_cmd_direct && (rx_kind != RX_NONE) &&
                              (tx_kind == TX_NONE) &&
                              (transfer_count == 2'd0) &&
                              (core_cmd_fop == last_fop);
wire core_cmd_idle = ((rx_kind == RX_NONE) || restartable_rx_command) &&
                   (tx_kind == TX_NONE) &&
                   (transfer_count == 2'd0) &&
                   !command_pending && !stack_write_a && !stack_write_b &&
                   !push_pending &&
                   (!memory_math_pending || restartable_rx_command) &&
                   !store_pending && !bcd_convert_pending && !bcd_load_pending &&
                   !pop_pending &&
                   !result_write_pending &&
                   !v2_exec_pending &&
                   !v2_exec_busy && !v2_exec_done &&
                   !fptan_trans_pending && !fptan_div_pending &&
                   !fptan_push_after_result &&
                   !poly_active &&
                   !read_resp_valid;
// Taken behind a running plain operation: register forms but the status
// reads (DF E0-FF), which need the finished status, and direct memory loads
// and arithmetic, whose operand enters the empty transfer queue. Bridge
// memory forms (stores, the original routines' transfers) wait as before.
wire core_cmd_holdable = core_cmd_direct ||
                         ((core_cmd_fop[7:6] == 2'b11) &&
                          !((core_cmd_fop[10:8] == 3'd7) && core_cmd_fop[5]));
// With every exception masked the running operation cannot raise one, so it
// retires as decoded: its result, its pops. Not in its completing cycle,
// whose stack write owns the port addresses.
wire core_cmd_behind = exec_plain && !v2_exec_done && queue_safe &&
                       !command_pending && !stack_write_a && !stack_write_b &&
                       (rx_kind == RX_NONE) && (tx_kind == TX_NONE) &&
                       (transfer_count == 2'd0) && (direct_tail_words == 2'd0) &&
                       !read_resp_valid && core_cmd_holdable;
wire core_cmd_ready = core_cmd_idle || core_cmd_behind;
// TOP the accepted command sees: after the running operation's pops.
wire [2:0] accept_top = top + (exec_plain ? {1'b0, arith_pop_count} : 3'd0);
wire [2:0] accept_row_b = ((fop_decode_key(core_cmd_fop) == 11'h530) ||
                           (core_cmd_fop == 11'h1f1) ||   // FYL2X
                           (core_cmd_fop == 11'h1f3) ||   // FPATAN
                           (core_cmd_fop == 11'h1f5) ||   // FPREM1
                           (core_cmd_fop == 11'h1f8) ||   // FPREM
                           (core_cmd_fop == 11'h1f9) ||   // FYL2XP1
                           (core_cmd_fop == 11'h1fd))     // FSCALE
                        ? accept_top + 3'd1 : accept_top + core_cmd_fop[2:0];
// A held command dispatches once its predecessor has retired (a stack RAM
// write may still be in flight: forwarded) and its reads are current;
// otherwise it reads its rows again first.
wire held_retired = !v2_exec_pending && !v2_exec_busy && !v2_exec_done &&
                    !result_write_pending && !push_pending && !pop_pending &&
                    !stack_write_b;
wire command_go = command_pending &&
                  (!command_held || (held_retired && command_rows_fresh));
// Direct memory commands remain ordered ahead of later bridge commands. Only
// masked-exception mode permits capture while a predecessor is still active.
assign direct_ready = !direct_pending && !cmd_valid && queue_safe;
assign cmd_ready = !direct_pending && core_cmd_ready;
assign quiet = !direct_pending && core_cmd_idle;
// A command waiting in the bridge is taken behind the running plain operation
// (at most a divide's 62 cycles), or once it and a held command that is
// quick or plain itself have dispatched (the held command may be dispatching
// now): a register command may wait for the bridge then.
wire held_short = (command_decode.action == X87_CMD_ARITH) ||
                  (command_decode.action == X87_CMD_FSQRT) ||
                  (command_decode.action == X87_CMD_FRNDINT) ||
                  (command_decode.action == X87_CMD_FSCALE) ||
                  (command_decode.action == X87_CMD_MEMORY_MATH) ||
                  (command_decode.action == X87_CMD_LOAD) ||
                  (command_decode.action == X87_CMD_FLD_ST) ||
                  (command_decode.action == X87_CMD_FXCH) ||
                  (command_decode.action == X87_CMD_FSTP_ST) ||
                  (command_decode.action == X87_CMD_FFREE) ||
                  (command_decode.action == X87_CMD_FCHS) ||
                  (command_decode.action == X87_CMD_FABS) ||
                  (command_decode.action == X87_CMD_FXAM) ||
                  (command_decode.action == X87_CMD_PUSH_CONST);
assign queue_short = (exec_plain || command_held) && (!command_pending || held_short);
// A direct operand's later fragments own the transfer queue's input.
assign word_in_ready = (rx_kind != RX_NONE) && (direct_tail_words == 2'd0) &&
                       transfer_push_ready;
assign read_req_ready = !read_resp_valid &&
                        (!read_req_data_port ||
                         ((tx_kind != TX_NONE) && transfer_pop_valid));
// A direct store takes its result fragments whole, low part first, as the
// 80386 reads them from the data port, without the bridge round trip. Only
// the store that produced them is waiting for them.
assign result_valid = (tx_kind == TX_VALUE) && transfer_pop_valid;
assign result_data = transfer_pop_data[31:0];
// FNSTSW and FNSTCW read these as the f8 port read returns them.
assign status_out = status_word;
assign control_out = control_word;
wire [2:0] rx_fragment_bytes = transfer_byte_count(transfer_pop_data[35:32]);
wire [3:0] rx_byte_count_next = rx_byte_count + rx_fragment_bytes;
// The 80386 sends a numeric operand low part first in whole 16-bit words
// (dwords, or words from 16-bit code and the m80 tail), so fragments land at
// even byte offsets with their bytes in the low lanes.
wire [79:0] rx_payload_next = place_transfer_word(
    rx_payload, rx_byte_count, transfer_pop_data[31:0],
    transfer_pop_data[35:32]);
// FSAVE packs register pairs at stream indices 6, 11, 16 and 21.
wire [2:0] fsave_pair_row = top + ((tx_index == 5'd11) ? 3'd2 :
                                   (tx_index == 5'd16) ? 3'd4 :
                                   (tx_index == 5'd21) ? 3'd6 : 3'd0);
// synthesis translate_off
always_ff @(posedge clk) begin
    if (!reset && core_cmd_valid && core_cmd_ready && (direct_tail_words != 2'd0))
        $error("x87 command accepted while a direct operand is still entering the queue");
    if (!reset && (direct_tail_words != 2'd0) && !transfer_push_ready)
        $error("x87 direct operand fragment found the transfer queue full");
    if (!reset && transfer_pop_valid && (rx_kind != RX_NONE) &&
        (rx_kind != RX_ENV) && (rx_kind != RX_STATE) &&
        (rx_byte_count[0] ||
         !((transfer_pop_data[35:32] == 4'hf) || (transfer_pop_data[35:32] == 4'h3))))
        $error("x87 operand fragment be=%h at byte %0d is not a low-lane dword/word",
               transfer_pop_data[35:32], rx_byte_count);
end
// synthesis translate_on
wire [2:0] tx_entry_bytes = transfer_byte_count(transfer_pop_data[35:32]);
wire [2:0] tx_request_bytes = transfer_byte_count(read_req_be);
wire tx_entry_consumed = tx_byte_offset + tx_request_bytes >= tx_entry_bytes;
wire v2_exec_start = v2_exec_pending && !v2_exec_busy;
logic [71:0] bcd_adjusted;
integer bcd_digit;
// FBSTP converts in place in the idle transmit shift register: digits in
// [135:64], the binary magnitude in [63:0], one double-dabble step a clock.
wire [71:0] bcd_digits = tx_state_shift[135:64];
always_comb begin
    bcd_adjusted = bcd_digits;
    for (bcd_digit = 0; bcd_digit < 18; bcd_digit = bcd_digit + 1)
        if (bcd_digits[bcd_digit*4 +: 4] >= 4'd5)
            bcd_adjusted[bcd_digit*4 +: 4] =
                bcd_digits[bcd_digit*4 +: 4] + 4'd3;
end
// FBLD runs the reverse in the same register: shift the digits right into
// the binary half, then take 3 from each digit of 8 or more.
logic [71:0] bcd_load_adjusted;
wire [71:0] bcd_load_digits = {1'b0, tx_state_shift[135:65]};
always_comb begin
    bcd_load_adjusted = bcd_load_digits;
    for (bcd_digit = 0; bcd_digit < 18; bcd_digit = bcd_digit + 1)
        if (bcd_load_digits[bcd_digit*4 +: 4] >= 4'd8)
            bcd_load_adjusted[bcd_digit*4 +: 4] =
                bcd_load_digits[bcd_digit*4 +: 4] - 4'd3;
end
assign busy_n = !(((v2_exec_owner == EXEC_MATH) &&
                   (v2_exec_pending || v2_exec_busy)) ||
                  poly_active ||
                  bcd_convert_pending || bcd_load_pending ||
                  (command_pending && command_is_arithmetic));
// PEREQ releases the 80386 coprocessor-wait microcode as well as requesting
// operand transfers. Keep it asserted throughout an accepted command because
// a one-cycle completion pulse can precede the CPU's CORWAIT sample.
assign pereq = (rx_kind != RX_NONE) || poly_active ||
               ((tx_kind != TX_NONE) && transfer_pop_valid) ||
               direct_pending || command_pending ||
               stack_write_a || stack_write_b ||
               push_pending || memory_math_pending ||
               store_pending || bcd_convert_pending || bcd_load_pending ||
               pop_pending || result_write_pending ||
               v2_exec_pending || v2_exec_busy || v2_exec_done ||
               fptan_trans_pending || fptan_div_pending ||
               fptan_push_after_result ||
               status_read_pending || command_complete_pulse ||
               (pereq_release_hold != 2'b00);
assign error_n = !status_flags[7];
assign queue_safe = &control_word[5:0] && error_n;
// A mask load from the command port to the receive that completes it. The CPU
// takes a direct load only when queue_safe cannot fall before its operand is
// queued: with every exception masked, nothing else can raise ES. A direct
// FLDCW (taken only with every exception masked) whose control word masks
// every exception again cannot make queue_safe fall: it is no mask load here.
assign mask_load_pending = (cmd_valid && fop_loads_masks(cmd_fop)) ||
                           (direct_pending && fop_loads_masks(direct_fop_r) &&
                            (direct_data_r[5:0] != 6'h3f)) ||
                           (command_pending && fop_loads_masks(command_fop) &&
                            !masked_cw_load_r) ||
                           ((rx_kind == RX_CONTROL) && !masked_cw_load_r) ||
                           (rx_kind == RX_ENV) || (rx_kind == RX_STATE);

// One physical three-word queue serves both protocol directions. A command
// cannot change direction until the queue is empty.
always_comb begin
    tx_produce_valid = (tx_kind != TX_NONE) && !tx_generation_done;
    tx_produce_data = 36'h0;
    case (tx_kind)
        TX_VALUE: begin
            tx_produce_data[31:0] = tx_state_shift[31:0];
            tx_produce_data[35:32] =
                (tx_index + 5'd1 == {3'h0, tx_count}) ? tx_last_be : 4'hf;
        end
        TX_ENV, TX_STATE: begin
            // The environment fields are architecturally 16 bits even when
            // the 32-bit format leaves two padding bytes after each field.
            tx_produce_data[35:32] = (tx_index < 5'd7) ? 4'h3 : 4'hf;
            if (tx_index == 5'd0)
                tx_produce_data[31:0] = {16'h0, control_word};
            else if (tx_index == 5'd1)
                tx_produce_data[31:0] = {16'h0, status_word};
            else if (tx_index == 5'd2)
                tx_produce_data[31:0] =
                    {16'h0, architectural_tag_word()};
            else if (tx_index < 5'd7)
                tx_produce_data[31:0] = 32'h0;
            else
                tx_produce_data[31:0] = tx_state_shift[31:0];
        end
        default: ;
    endcase

    // A direct operand enters the queue low fragment first, from the cycle
    // its command is accepted: the queue is empty then and holds all three
    // fragments of an m80. A two-byte last fragment has its bytes in the low
    // lanes, as the 80386 sends an m80 tail.
    if (core_cmd_direct && core_cmd_valid && core_cmd_ready) begin
        transfer_push_valid = 1'b1;
        transfer_push_data = {((direct_words_r == 2'd1) && direct_tail16_r) ? 4'h3 : 4'hf,
                              direct_data_r[31:0]};
    end else if (direct_tail_words != 2'd0) begin
        transfer_push_valid = 1'b1;
        transfer_push_data = {((direct_tail_words == 2'd1) && direct_tail16_r) ? 4'h3 : 4'hf,
                              direct_tail_data[31:0]};
    end else begin
        transfer_push_valid = (rx_kind != RX_NONE)
                            ? word_in_valid : tx_produce_valid;
        transfer_push_data = (rx_kind != RX_NONE)
                           ? {word_in_be, word_in_data} : tx_produce_data;
    end
    transfer_pop_ready = (rx_kind != RX_NONE) ? 1'b1
                       : ((tx_kind != TX_NONE) && read_req_valid &&
                          read_req_ready && read_req_data_port &&
                          tx_entry_consumed) ||
                         (result_pop && (tx_kind == TX_VALUE));
end

assign tx_produce_fire = (tx_kind != TX_NONE) &&
                         transfer_push_valid && transfer_push_ready;
assign tx_consume_fire = (tx_kind != TX_NONE) &&
                         transfer_pop_valid && transfer_pop_ready;

// Command decode and stack operands are synchronous and return together one
// cycle after command acceptance. This keeps FOP classification out of the
// command-dispatch datapath without adding a protocol cycle.
x87_command_rom command_rom (
    .clk(clk),
    .address(command_pending ? command_fop : core_cmd_fop),
    .decode(command_decode_rom)
);

`ifdef X87_ABLATE
// Area measurement only: X87_ABLATE is a feature bitmask whose commands and
// executor operations are decoded as absent, so synthesis prunes their logic.
localparam int X87_ABLATE_MASK = `X87_ABLATE;
function automatic logic ablated_action(input x87_command_action_t action);
    return (X87_ABLATE_MASK[0] && (action == X87_CMD_STORE_BCD)) ||
           (X87_ABLATE_MASK[1] && (action == X87_CMD_FSCALE)) ||
           (X87_ABLATE_MASK[2] && (action == X87_CMD_FPREM)) ||
           (X87_ABLATE_MASK[3] && ((action == X87_CMD_FYL2X) ||
                                   (action == X87_CMD_F2XM1))) ||
           (X87_ABLATE_MASK[4] && ((action == X87_CMD_FPTAN) ||
                                   (action == X87_CMD_FPATAN) ||
                                   (action == X87_CMD_TRIG))) ||
           (X87_ABLATE_MASK[5] && ((action == X87_CMD_TX_ENV) ||
                                   (action == X87_CMD_TX_STATE) ||
                                   (action == X87_CMD_RX_ENV) ||
                                   (action == X87_CMD_RX_STATE)));
endfunction
function automatic logic ablated_exec_op(input x87_exec_op_t op);
    return (X87_ABLATE_MASK[4] && (op == X87_ARITH_TRANS)) ||
           (X87_ABLATE_MASK[6] && ((op == X87_ARITH_DIV) || (op == X87_ARITH_SQRT))) ||
           (X87_ABLATE_MASK[7] && (op == X87_ARITH_MUL)) ||
           (X87_ABLATE_MASK[8] && ((op == X87_CONVERT_FILD) || (op == X87_CONVERT_FIST))) ||
           (X87_ABLATE_MASK[9] && ((op == X87_CONVERT_FLD_M32) || (op == X87_CONVERT_FLD_M64) ||
                                   (op == X87_CONVERT_FST_M32) || (op == X87_CONVERT_FST_M64))) ||
           (X87_ABLATE_MASK[1] && (op == X87_ARITH_SCALE)) ||
           (X87_ABLATE_MASK[10] && (op == X87_CONVERT_FRNDINT)) ||
           (X87_ABLATE_MASK[11] && ((op == X87_ARITH_ADD) || (op == X87_ARITH_SUB)));
endfunction
always_comb begin
    command_decode = command_decode_rom;
    if (ablated_action(command_decode_rom.action))
        command_decode.action = X87_CMD_NONE;
end
assign v2_exec_op_port = ablated_exec_op(v2_exec_op) ? X87_ARITH_COMPARE : v2_exec_op;
`else
assign command_decode = command_decode_rom;
assign v2_exec_op_port = v2_exec_op;
`endif

x87_transfer_fifo transfer_fifo (
    .clk(clk),
    .reset(reset),
    .clear(1'b0),
    .push_valid(transfer_push_valid),
    .push_data(transfer_push_data),
    .push_ready(transfer_push_ready),
    .pop_valid(transfer_pop_valid),
    .pop_data(transfer_pop_data),
    .pop_ready(transfer_pop_ready),
    .count(transfer_count)
);

// A newly accepted command directly addresses the synchronous read ports. The
// registered addresses retain that selection for writes and multiword state.
always_comb begin
    stack_port_addr_a = stack_addr_a;
    stack_port_addr_b = stack_addr_b;
    if (core_cmd_valid && core_cmd_ready) begin
        stack_port_addr_a = accept_top;
        stack_port_addr_b = accept_row_b;
    end
end

x87_stack_mem stack_mem (
    .clk(clk),
    .addr_a(stack_port_addr_a),
    .write_a(stack_write_a),
    .write_data_a(stack_write_data_a),
    .read_data_a(stack_ram_q_a),
    .addr_b(stack_port_addr_b),
    .write_b(stack_write_b),
    .write_data_b(stack_write_data_b),
    .read_data_b(stack_ram_q_b)
);

// XTR's two results: m (with A's guard, round and sticky) and 2^27 + 2^15 + k.
x87_reg_t xtr_m, xtr_t;
always_comb begin
    xtr_m = '0;
    xtr_m.exp = xtr_ge ? 15'h3ffe : 15'h3fff;
    xtr_m.sig = xtr_sig;
    xtr_m.guard_bit = xtr_guard;
    xtr_m.round_bit = xtr_round;
    xtr_m.sticky_bit = xtr_sticky;
    xtr_m.class_id = X87_NORMAL;
    xtr_t = '0;
    xtr_t.exp = 15'h3fff + 15'd27;
    xtr_t.sig = {1'b1, 11'h0, xtr_u, 25'h0};
    xtr_t.class_id = X87_NORMAL;
end
// Scratch writes: the two inputs when a program starts, then each result.
always_comb begin
    poly_we = 1'b0;
    poly_waddr = 4'd0;
    poly_wdata = arith_operand_a;
    if (trans_compute) begin
        poly_we = 1'b1;
    end else if (poly_phase == POLY_LOAD_B) begin
        poly_we = 1'b1;
        poly_waddr = 4'd1;
        poly_wdata = arith_operand_b;
    end else if ((poly_phase == POLY_WAIT) && v2_exec_math_done && !poly_final &&
                 (poly_word.op != X87_POLY_OP_CMP)) begin
        poly_we = 1'b1;
        poly_waddr = poly_word.dest;
        poly_wdata = v2_exec_result;
    end else if (poly_phase == POLY_XTR_M) begin
        poly_we = 1'b1;
        poly_waddr = poly_word.dest;
        poly_wdata = xtr_m;
    end else if (poly_phase == POLY_XTR_K) begin
        poly_we = 1'b1;
        poly_waddr = poly_word.b_index[3:0];
        poly_wdata = xtr_t;
    end
end
// Operand reads: A while the predicate is checked, B the cycle after; while
// an operation runs, the next word's (pf_word): A on its port and B on the
// other, then B if both use one port.
x87_poly_word_t poly_read_word;
wire poly_fetch_reads = (poly_phase == POLY_WAIT) && (pf_state == PF_PRED);
wire poly_read_b = (poly_phase == POLY_FETCH_B) ||
                   ((poly_phase == POLY_WAIT) && (pf_state == PF_CAPA));
assign poly_read_word = (poly_phase == POLY_WAIT) ? pf_word : poly_word;
wire [6:0] poly_read_ca = poly_read_word.a_index +
                          (poly_read_word.a_id ? {5'd0, poly_id} : 7'd0);
wire [6:0] poly_read_cb = poly_read_word.b_index +
                          (poly_read_word.b_id ? {5'd0, poly_id} : 7'd0);
always_comb begin
    if (poly_read_b) begin
        poly_saddr = poly_read_word.b_index[3:0];
        poly_caddr = poly_read_cb;
    end else if (poly_fetch_reads && poly_read_word.a_const) begin
        poly_saddr = poly_read_word.b_index[3:0];
        poly_caddr = poly_read_ca;
    end else if (poly_fetch_reads) begin
        poly_saddr = poly_read_word.a_index[3:0];
        poly_caddr = poly_read_cb;
    end else begin
        poly_saddr = poly_read_word.a_index[3:0];
        poly_caddr = poly_read_ca;
    end
end
always_ff @(posedge clk) begin
    if (poly_we)
        poly_scratch[poly_waddr] <= poly_wdata;
    poly_scratch_q <= poly_scratch[poly_saddr];
    poly_const_q <= x87_reg_t'(x87_poly_const(poly_caddr));
end
x87_reg_t poly_opb_read;
x87_reg_t poly_opa_read;
logic [2:0] poly_round_low3;               // n mod 8 of the RND result's magnitude
assign poly_opb_read = poly_word.b_const ? poly_const_q : poly_scratch_q;
x87_reg_t pf_opa_read, pf_opb_read;
assign pf_opa_read = pf_word.a_const ? poly_const_q : poly_scratch_q;
assign pf_opb_read = pf_word.b_const ? poly_const_q : poly_scratch_q;
// The fetched word reads the slot the running operation writes at done.
wire pf_forward_a = !pf_word.a_const && (pf_word.a_index[3:0] == poly_word.dest);
wire pf_forward_b = !pf_word.b_const && (pf_word.b_index[3:0] == poly_word.dest);
// Operand modifiers: truncation to the top 13 significand bits (the
// programs' exact products), then |A| or -A, |B|.
function automatic x87_reg_t poly_a_sign(input x87_reg_t value, input x87_poly_word_t word);
    x87_reg_t r;
    r = value;
    if (word.a_trunc) begin
        r.sig[39:0] = 40'h0;
        r.guard_bit = 1'b0;
        r.round_bit = 1'b0;
        r.sticky_bit = 1'b0;
    end
    if (word.a_abs)
        r.sign = 1'b0;
    else if (word.a_neg)
        r.sign = !value.sign;
    return r;
endfunction
function automatic x87_reg_t poly_b_sign(input x87_reg_t value, input x87_poly_word_t word);
    x87_reg_t r;
    r = value;
    if (word.b_trunc) begin
        r.sig[39:0] = 40'h0;
        r.guard_bit = 1'b0;
        r.round_bit = 1'b0;
        r.sticky_bit = 1'b0;
    end
    if (word.b_abs)
        r.sign = 1'b0;
    return r;
endfunction
function automatic x87_exec_op_t poly_exec_op(input logic [2:0] op);
    case (op)
        X87_POLY_OP_ADD: return X87_ARITH_ADD;
        X87_POLY_OP_SUB: return X87_ARITH_SUB;
        X87_POLY_OP_DIV: return X87_ARITH_DIV;
        X87_POLY_OP_RND: return X87_CONVERT_FRNDINT;
        X87_POLY_OP_CMP: return X87_ARITH_COMPARE;
        default:         return X87_ARITH_MUL;
    endcase
endfunction
// A word after which the next one is fetched while it runs.
function automatic logic poly_fetch_behind(input x87_poly_word_t word);
    return !word.final_op && (word.op != X87_POLY_OP_CMP) &&
           (word.op != X87_POLY_OP_RND);
endfunction
// The word after the one issuing (ISSUE) or after pf_word (issue at done, a
// skipped word), read as an operation issues.
wire [7:0] pf_next_pc = (poly_phase == POLY_ISSUE) ? poly_pc + 8'd1 : pf_pc + 8'd1;
x87_poly_word_t pf_next_word;
assign pf_next_word = x87_poly_word_t'(x87_poly_word(pf_next_pc));
assign poly_opa_read = poly_word.a_const ? poly_const_q : poly_scratch_q;
assign poly_round_low3 = x87_integer_low3(v2_exec_result);

wire [1:0] executor_rounding_mode =
    poly_internal                ? 2'b00 :
    (fprem_phase == FPREM_ROUND) ? (fprem_nearest ? 2'b00 : 2'b11)
                                 : control_word[11:10];

x87_executor executor (
    .clk(clk),
    .reset(reset),
    .start(v2_exec_start),
    .exec_op(v2_exec_op_port),
    .integer_size(v2_exec_size),
    .precision_control(poly_internal ? 2'b11 : control_word[9:8]),
    .rounding_mode(executor_rounding_mode),
    .quiet_compare(arith_quiet_compare),
    .trans_cosine(trans_cosine),
    .trans_atan2(trans_atan2),
    .trans_logexp(trans_logexp),
    .range_masks(control_word[4:3]),
    .operand(arith_operand_a),
    .operand_b(arith_operand_b),
    .transfer_in(v2_exec_transfer),
    .busy(v2_exec_busy),
    .done(v2_exec_done),
    .commit_action(v2_exec_commit),
    .result(v2_exec_result),
    .transfer_out(v2_exec_transfer_out),
    .invalid(v2_exec_invalid),
    .inexact(v2_exec_inexact),
    .divide_by_zero(v2_exec_divide_by_zero),
    .overflow(v2_exec_overflow),
    .underflow(v2_exec_underflow),
    .denormal_operand(v2_exec_denormal_operand),
    .range_incomplete(v2_exec_range_incomplete),
    .rounded_up(v2_exec_rounded_up),
    .compare_unordered(v2_compare_unordered),
    .compare_less(v2_compare_less),
    .compare_equal(v2_compare_equal),
    .debug_uaddr(v2_exec_uaddr)
);

always_ff @(posedge clk) begin
    tag_full_we = 1'b0;
    tag_full_value = 16'h0;
    tag_a_we = 1'b0;
    tag_a_index = 3'd0;
    tag_a_value = 2'b00;
    tag_b_we = 1'b0;
    tag_b_index = 3'd0;
    tag_b_value = 2'b00;
    tag_pop_count = 2'd0;
    if (reset) begin
        control_word <= 16'h037f;
        status_flags <= 16'h0000;
        top <= 3'd0;
        last_fop <= 11'h000;
        command_pending <= 1'b0;
        command_fop <= 11'h000;
        exec_plain <= 1'b0;
        command_held <= 1'b0;
        command_row_a <= 3'd0;
        command_row_b <= 3'd1;
        command_rows_fresh <= 1'b0;
        direct_pending <= 1'b0;
        direct_fop_r <= 11'h000;
        direct_data_r <= 80'h0;
        direct_words_r <= 2'd1;
        direct_tail16_r <= 1'b0;
        masked_cw_load_r <= 1'b0;
        direct_tail_words <= 2'd0;
        direct_tail_data <= 48'h0;
        stack_addr_a <= 3'd0;
        stack_addr_b <= 3'd1;
        stack_write_a <= 1'b0;
        stack_write_b <= 1'b0;
        stack_write_data_a <= 80'h0;
        stack_write_data_b <= 80'h0;
        rx_kind <= RX_NONE;
        rx_index <= 5'd0;
        rx_payload <= 80'h0;
        rx_byte_count <= 4'd0;
        rx_state_shift <= '0;
        tx_kind <= TX_NONE;
        tx_count <= 2'd0;
        tx_last_be <= 4'hf;
        tx_index <= 5'd0;
        tx_generation_done <= 1'b0;
        tx_byte_offset <= 3'd0;
        tx_state_shift <= '0;
        status_read_pending <= 1'b0;
        command_complete_pulse <= 1'b0;
        pereq_release_hold <= 2'b00;
        push_pending <= 1'b0;
        push_pending_raw <= 80'h0;
        push_pending_tag <= 2'b11;
        memory_math_pending <= 1'b0;
        memory_math_mul <= 1'b0;
        memory_math_div <= 1'b0;
        memory_math_compare <= 1'b0;
        memory_math_subtract <= 1'b0;
        memory_math_reverse <= 1'b0;
        memory_math_pop <= 1'b0;
        memory_math_st0 <= x87_empty();
        store_pending <= 1'b0;
        store_integer <= 1'b0;
        store_width <= 2'd0;
        store_pop <= 1'b0;
        store_source_empty <= 1'b0;
        store_source <= x87_empty();
        store_source_raw <= 80'h0;
        store_bcd <= 1'b0;
        bcd_convert_pending <= 1'b0;
        bcd_shift_count <= 7'd0;
        bcd_sign <= 1'b0;
        bcd_load_pending <= 1'b0;
        bcd_load_negate <= 1'b0;
        pop_pending <= 1'b0;
        result_write_pending <= 1'b0;
        result_write_index <= 3'd0;
        result_write_raw <= 80'h0;
        v2_exec_pending <= 1'b0;
        v2_exec_op <= X87_CONVERT_FLD_M32;
        v2_exec_owner <= EXEC_NONE;
        v2_exec_size <= 2'd0;
        v2_exec_transfer <= 64'h0;
        arith_compare <= 1'b0;
        arith_quiet_compare <= 1'b0;
        arith_write_result <= 1'b0;
        arith_pop_count <= 2'd0;
        arith_dest_index <= 3'd0;
        arith_operand_a <= x87_empty();
        arith_operand_b <= x87_empty();
        trans_cosine <= 1'b0;
        trans_atan2 <= 1'b0;
        trans_logexp <= X87_TRANS_NONE;
        fptan_trans_pending <= 1'b0;
        fptan_div_pending <= 1'b0;
        fptan_push_after_result <= 1'b0;
        fptan_sincos <= 1'b0;
        fptan_push_value <= x87_one();
        poly_phase <= POLY_IDLE;
        poly_pc <= 8'd0;
        poly_word <= '0;
        pf_state <= PF_IDLE;
        pf_pc <= 8'd0;
        pf_word <= '0;
        pf_opa <= x87_empty();
        pf_opb <= x87_empty();
        poly_quadrant <= 2'd0;
        poly_swap <= 1'b0;
        poly_id <= 2'd0;
        poly_xneg <= 1'b0;
        poly_yneg <= 1'b0;
        poly_cosine <= 1'b0;
        poly_final <= 1'b0;
        poly_denormal <= 1'b0;
        poly_k0 <= 1'b1;
        poly_exact <= 1'b0;
        xtr_sig <= 53'h0;
        xtr_guard <= 1'b0;
        xtr_round <= 1'b0;
        xtr_sticky <= 1'b0;
        xtr_exp <= 17'sd0;
        xtr_ge <= 1'b0;
        xtr_u <= 16'h0;
        poly_aux <= x87_one();
        poly_opa <= x87_empty();
        fprem_phase <= FPREM_IDLE;
        fprem_nearest <= 1'b0;
        fxtract_exp_pending <= 1'b0;
        fxtract_exp_index <= 3'd0;
        fprem_dividend <= x87_empty();
        fprem_divisor <= x87_empty();
        fprem_quotient_bits <= 3'b000;
        read_resp_valid <= 1'b0;
        read_resp_data <= 32'h0;
        clear_stack();
    end else begin
        read_resp_valid <= 1'b0;
        command_complete_pulse <= 1'b0;
        stack_write_a <= 1'b0;
        stack_write_b <= 1'b0;
        pereq_release_hold <= {1'b0, pereq_release_hold[1]};

        if (direct_valid && direct_ready) begin
            direct_pending <= 1'b1;
            direct_fop_r <= direct_fop;
            direct_data_r <= direct_data;
            direct_words_r <= direct_words;
            direct_tail16_r <= direct_tail16;
        end
        if ((direct_tail_words != 2'd0) && transfer_push_ready) begin
            direct_tail_words <= direct_tail_words - 2'd1;
            direct_tail_data <= {32'h0, direct_tail_data[47:32]};
        end

        // Transfer conversion and TOP-dependent stack selection are separate
        // cycles. PEREQ keeps the CPU stalled until this commit completes.
        if (push_pending) begin
            if (memory_math_pending) begin
                launch_memory_math(push_pending_value);
            end else begin
                push_raw_tagged(push_pending_raw, push_pending_tag);
            end
            push_pending <= 1'b0;
        end

        if (store_pending) begin
            if (store_bcd) begin
                v2_exec_op <= X87_CONVERT_FIST;
                v2_exec_owner <= EXEC_STORE;
                v2_exec_size <= 2'd2;
                arith_operand_a <= store_source;
                v2_exec_transfer <= 64'h0;
                v2_exec_pending <= 1'b1;
            end else if (!store_integer && (store_width == 2'd2)) begin
                logic [79:0] store_m80_value;
                store_m80_value = store_source_raw;
                tx_state_shift <= {80'h0, store_m80_value};
                tx_count <= 2'd3;
                tx_last_be <= 4'h3;
                tx_index <= 5'd0;
                tx_kind <= TX_VALUE;
                tx_generation_done <= 1'b0;
                if (store_pop && (!store_source_empty || control_word[0]))
                    pop_pending <= 1'b1;
            end else begin
                v2_exec_op <= store_integer ? X87_CONVERT_FIST
                              : (store_width == 2'd0)
                              ? X87_CONVERT_FST_M32
                              : X87_CONVERT_FST_M64;
                v2_exec_owner <= EXEC_STORE;
                v2_exec_size <= store_width;
                arith_operand_a <= store_source;
                v2_exec_transfer <= 64'h0;
                v2_exec_pending <= 1'b1;
            end
            store_pending <= 1'b0;
        end

        if (v2_exec_start) begin
            v2_exec_pending <= 1'b0;
        end

        if (v2_exec_done)
            exec_plain <= 1'b0;

        if (v2_exec_done) begin
            case (v2_exec_commit)
                X87_COMMIT_PUSH: begin
                    if (fxtract_exp_pending) begin
                        // FXTRACT's exponent, converted by FILD, replaces
                        // the operand under the pushed significand.
                        fxtract_exp_pending <= 1'b0;
                        result_write_index <= fxtract_exp_index;
                        result_write_raw <= x87_to_m80(v2_exec_result);
                        result_write_pending <= 1'b1;
                        command_complete_pulse <= 1'b1;
                    end else begin
                        x87_reg_t loaded;
                        loaded = v2_exec_result;
                        if (bcd_load_negate) begin
                            loaded.sign = 1'b1;   // FBLD of -0 loads -0
                            bcd_load_negate <= 1'b0;
                        end
                        if (memory_math_pending) begin
                            // A converted memory operand: the push stage
                            // pairs it with ST0 next cycle.
                            push_pending_raw <= x87_to_m80(loaded);
                            push_pending_tag <= x87_tag(loaded);
                            push_pending <= 1'b1;
                        end else begin
                            // A load pushes as it completes.
                            push_raw_tagged(x87_to_m80(loaded), x87_tag(loaded));
                        end
                    end
                    if (v2_exec_invalid)
                        raise_invalid();
                end
                X87_COMMIT_TRANSFER: begin
                    // FIST/FST/FBSTP report conversion rounding in C1, just
                    // like arithmetic. Invalid/empty stores keep C1 clear.
                    status_flags[9] <= !store_source_empty &&
                                       !v2_exec_invalid && v2_exec_rounded_up;
                    if (store_bcd) begin
                        logic [63:0] bcd_magnitude;
                        bcd_magnitude = v2_exec_transfer_out[63]
                                      ? (~v2_exec_transfer_out + 64'd1)
                                      : v2_exec_transfer_out;
                        if (v2_exec_invalid ||
                            (bcd_magnitude > 64'd999999999999999999)) begin
                            tx_state_shift <= {
                                80'h0, 80'hffff_c000_0000_0000_0000};
                            tx_count <= 2'd3;
                            tx_last_be <= 4'h3;
                            tx_index <= 5'd0;
                            tx_kind <= TX_VALUE;
                            tx_generation_done <= 1'b0;
                            store_bcd <= 1'b0;
                            if (!v2_exec_invalid)
                                raise_invalid();
                            if (store_pop && control_word[0])
                                pop_pending <= 1'b1;
                        end else begin
                            tx_state_shift <= {96'h0, bcd_magnitude};
                            bcd_shift_count <= 7'd64;
                            bcd_sign <= v2_exec_transfer_out[63];
                            bcd_convert_pending <= 1'b1;
                        end
                    end else begin
                        tx_state_shift <= {96'h0, v2_exec_transfer_out};
                        tx_count <= ((store_integer && (store_width == 2'd2)) ||
                                     (!store_integer && (store_width == 2'd1)))
                                  ? 2'd2 : 2'd1;
                        tx_last_be <= (store_integer && (store_width == 2'd0))
                                    ? 4'h3 : 4'hf;
                        tx_index <= 5'd0;
                        tx_kind <= TX_VALUE;
                        tx_generation_done <= 1'b0;
                    end
                    if (v2_exec_invalid)
                        raise_invalid();
                    if (v2_exec_overflow) begin
                        status_flags[3] <= 1'b1;
                        if (!control_word[3]) begin
                            status_flags[7] <= 1'b1;
                            status_flags[15] <= 1'b1;
                        end
                    end
                    if (v2_exec_underflow) begin
                        status_flags[4] <= 1'b1;
                        if (!control_word[4]) begin
                            status_flags[7] <= 1'b1;
                            status_flags[15] <= 1'b1;
                        end
                    end
                    if (v2_exec_inexact) begin
                        status_flags[5] <= 1'b1;
                        if (!control_word[5]) begin
                            status_flags[7] <= 1'b1;
                            status_flags[15] <= 1'b1;
                        end
                    end
                    if (!store_bcd && store_pop &&
                        (store_integer
                         ? (!(v2_exec_invalid || store_source_empty) ||
                            control_word[0])
                         : (!store_source_empty || control_word[0])))
                        pop_pending <= 1'b1;
                end
                default: ;
            endcase
        end

        if (bcd_convert_pending) begin
            tx_state_shift[135:0] <= {bcd_adjusted[70:0], tx_state_shift[63:0], 1'b0};
            bcd_shift_count <= bcd_shift_count - 7'd1;
            if (bcd_shift_count == 7'd1) begin
                tx_state_shift <= {
                    80'h0, {bcd_sign, 7'h00,
                            bcd_adjusted[70:0], tx_state_shift[63]}};
                tx_count <= 2'd3;
                tx_last_be <= 4'h3;
                tx_index <= 5'd0;
                tx_kind <= TX_VALUE;
                tx_generation_done <= 1'b0;
                bcd_convert_pending <= 1'b0;
                store_bcd <= 1'b0;
                if (store_pop && (!store_source_empty || control_word[0]))
                    pop_pending <= 1'b1;
            end
        end

        if (bcd_load_pending) begin
            tx_state_shift[135:0] <= {bcd_load_adjusted, tx_state_shift[64:1]};
            bcd_shift_count <= bcd_shift_count - 7'd1;
            if (bcd_shift_count == 7'd1) begin
                // 18 digits fit in 60 bits: FILD m64 of the magnitude.
                v2_exec_op <= X87_CONVERT_FILD;
                v2_exec_owner <= EXEC_LOAD;
                v2_exec_size <= 2'd2;
                v2_exec_transfer <= tx_state_shift[64:1];
                v2_exec_pending <= 1'b1;
                bcd_load_pending <= 1'b0;
            end
        end

        if (pop_pending) begin
            pop_value();
            pop_pending <= 1'b0;
        end

        if (result_write_pending) begin
            write_stack_raw(result_write_index, result_write_raw);
            result_write_pending <= 1'b0;
        end

        // FPTAN commits tan(x) to the old ST0 before pushing the architectural
        // 1.0 result. Keeping these as separate stack-RAM cycles avoids a
        // second write-port dependency on the normal result path.
        if (fptan_push_after_result && !result_write_pending &&
            !push_pending) begin
            schedule_push(fptan_push_value);
            fptan_push_after_result <= 1'b0;
        end

        if (trans_done && fptan_trans_pending) begin
            fptan_trans_pending <= 1'b0;
            status_flags[10] <= v2_exec_range_incomplete; // C2

            if (v2_exec_invalid)
                raise_invalid();
            if (trans_denormal_operand) begin
                status_flags[1] <= 1'b1;
                if (!control_word[1]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (v2_exec_range_incomplete) begin
                command_complete_pulse <= 1'b1;
            end else if (v2_exec_invalid) begin
                if (control_word[0]) begin
                    result_write_index <= arith_dest_index;
                    result_write_raw <= x87_to_m80(v2_exec_result);
                    result_write_pending <= 1'b1;
                    fptan_push_value <= fptan_sincos ? v2_exec_result : x87_one();
                    fptan_push_after_result <= 1'b1;
                    command_complete_pulse <= 1'b0;
                end
            end else if (trans_denormal_operand && !control_word[1]) begin
                command_complete_pulse <= 1'b1;
            end else if (fptan_sincos) begin
                // FSINCOS: sin(x) replaces ST0, then cos(x) is pushed.
                if (trans_inexact)
                    status_flags[5] <= 1'b1;
                result_write_index <= arith_dest_index;
                result_write_raw <= x87_to_m80(v2_exec_result);
                result_write_pending <= 1'b1;
                fptan_push_value <= trans_aux;
                fptan_push_after_result <= 1'b1;
                command_complete_pulse <= 1'b0;
            end else begin
                arith_operand_a <= v2_exec_result;
                arith_operand_b <= trans_aux;
                v2_exec_op <= X87_ARITH_DIV;
                v2_exec_owner <= EXEC_MATH;
                v2_exec_size <= 2'd0;
                v2_exec_transfer <= 64'h0;
                v2_exec_pending <= 1'b1;
                fptan_div_pending <= 1'b1;
                command_complete_pulse <= 1'b0;
            end
        end

        if (v2_exec_math_done && fptan_div_pending) begin
            fptan_div_pending <= 1'b0;
            if (!math_unmasked_exception)
                fptan_push_after_result <= 1'b1;
        end

        // FPREM reuses the existing divide, round-to-integer, multiply, and
        // subtract microprograms. The quotient is rounded toward zero, and
        // its low three bits become C0/C3/C1 after the remainder commits.
        if (v2_exec_math_done && (fprem_phase != FPREM_IDLE)) begin
            case (fprem_phase)
                FPREM_DIVIDE: begin
                    arith_operand_a <= v2_exec_result;
                    arith_operand_b <= x87_empty();
                    v2_exec_op <= X87_CONVERT_FRNDINT;
                    v2_exec_owner <= EXEC_MATH;
                    v2_exec_size <= 2'd0;
                    v2_exec_transfer <= 64'h0;
                    v2_exec_pending <= 1'b1;
                    fprem_phase <= FPREM_ROUND;
                end
                FPREM_ROUND: begin
                    fprem_quotient_bits <= x87_integer_low3(v2_exec_result);
                    arith_operand_a <= v2_exec_result;
                    arith_operand_b <= fprem_divisor;
                    v2_exec_op <= X87_ARITH_MUL;
                    v2_exec_owner <= EXEC_MATH;
                    v2_exec_size <= 2'd0;
                    v2_exec_transfer <= 64'h0;
                    v2_exec_pending <= 1'b1;
                    fprem_phase <= FPREM_MULTIPLY;
                end
                FPREM_MULTIPLY: begin
                    arith_operand_a <= fprem_dividend;
                    arith_operand_b <= v2_exec_result;
                    v2_exec_op <= X87_ARITH_SUB;
                    v2_exec_owner <= EXEC_MATH;
                    v2_exec_size <= 2'd0;
                    v2_exec_transfer <= 64'h0;
                    v2_exec_pending <= 1'b1;
                    fprem_phase <= FPREM_SUBTRACT;
                end
                FPREM_SUBTRACT: begin
                    fprem_phase <= FPREM_IDLE;
                    status_flags[14] <= fprem_quotient_bits[1]; // C3 = Q1
                    status_flags[10] <= 1'b0;                   // Complete
                    status_flags[9] <= fprem_quotient_bits[0]; // C1 = Q0
                    status_flags[8] <= fprem_quotient_bits[2]; // C0 = Q2
                    result_write_index <= st0_index;
                    result_write_raw <= x87_to_m80(v2_exec_result);
                    result_write_pending <= 1'b1;
                    command_complete_pulse <= 1'b0;
                end
                default: ;
            endcase
        end

        // Transcendental program: each word reads A and B, runs one executor
        // operation (round to nearest, 53 bits) and writes the scratch; the
        // final word rounds per the control word and retires as the TRANS
        // result (trans_done), with cos(x) in poly_aux. F2XM1 and FYL2XP1
        // of |x| < 2^-60 run their short forms.
        if (trans_compute) begin
            poly_phase <= POLY_LOAD_B;
            case (trans_logexp)
                X87_TRANS_F2XM1:
                    poly_pc <= (arith_operand_a.exp < 15'h3fc3)
                             ? X87_POLY_ENTRY_EXP_TINY : X87_POLY_ENTRY_EXP;
                X87_TRANS_FYL2X:
                    poly_pc <= X87_POLY_ENTRY_LOG;
                X87_TRANS_FYL2XP1:
                    poly_pc <= (arith_operand_b.exp < 15'h3fc3)
                             ? X87_POLY_ENTRY_LOG1P_TINY : X87_POLY_ENTRY_LOG1P;
                default:
                    poly_pc <= trans_atan2 ? X87_POLY_ENTRY_ATAN : X87_POLY_ENTRY_SINCOS;
            endcase
            poly_k0 <= 1'b1;
            poly_exact <= 1'b0;
            poly_quadrant <= 2'd0;
            poly_swap <= 1'b0;
            poly_id <= 2'd0;
            poly_yneg <= arith_operand_a.sign;       // FPATAN: A = Y (ST1)
            poly_xneg <= arith_operand_b.sign;       //         B = X (ST0)
            poly_cosine <= trans_cosine;
            poly_final <= 1'b0;
            poly_denormal <= v2_exec_denormal_operand;
            command_complete_pulse <= 1'b0;
            pf_state <= PF_IDLE;
        end
        case (poly_phase)
            POLY_LOAD_B: poly_phase <= POLY_DECODE;
            POLY_DECODE: begin
                poly_word <= x87_poly_word_t'(x87_poly_word(poly_pc));
                poly_phase <= POLY_FETCH_A;
            end
            POLY_FETCH_A: begin
                if (x87_poly_pred(poly_word.pred, poly_quadrant, poly_cosine,
                                  poly_swap, poly_xneg, poly_yneg, poly_id, poly_k0)) begin
                    poly_phase <= POLY_FETCH_B;
                end else begin
                    poly_pc <= poly_pc + 8'd1;
                    poly_phase <= POLY_DECODE;
                end
            end
            POLY_FETCH_B: begin
                poly_opa <= poly_a_sign(poly_opa_read, poly_word);
                poly_phase <= POLY_ISSUE;
            end
            POLY_ISSUE: begin
                if (poly_word.op == X87_POLY_OP_AUX) begin
                    poly_aux <= poly_opa;
                    poly_pc <= poly_pc + 8'd1;
                    poly_phase <= POLY_DECODE;
                end else if (poly_word.op == X87_POLY_OP_XTR) begin
                    // A positive normal or denormal (exponent 0: effectively 1).
                    xtr_sig <= poly_opa.sig;
                    xtr_guard <= poly_opa.guard_bit;
                    xtr_round <= poly_opa.round_bit;
                    xtr_sticky <= poly_opa.sticky_bit;
                    xtr_exp <= (poly_opa.class_id == X87_DENORMAL)
                             ? 17'sd1 : $signed({2'b00, poly_opa.exp});
                    poly_phase <= POLY_XTR_NORM;
                end else begin
                    arith_operand_a <= poly_opa;
                    arith_operand_b <= poly_b_sign(poly_opb_read, poly_word);
                    v2_exec_op <= poly_exec_op(poly_word.op);
                    v2_exec_owner <= EXEC_MATH;
                    v2_exec_size <= 2'd0;
                    v2_exec_transfer <= 64'h0;
                    v2_exec_pending <= 1'b1;
                    poly_final <= poly_word.final_op;
                    poly_phase <= POLY_WAIT;
                    pf_pc <= pf_next_pc;
                    pf_word <= pf_next_word;
                    pf_state <= poly_fetch_behind(poly_word) ? PF_PRED : PF_IDLE;
                end
            end
            POLY_WAIT: begin
                if (v2_exec_math_done) begin
                    if (poly_final) begin
                        poly_final <= 1'b0;
                        poly_phase <= POLY_IDLE;
                        pf_state <= PF_IDLE;
                    end else if (pf_state == PF_READY) begin
                        // The fetched word issues now (the scratch takes
                        // this result in the same cycle).
                        poly_word <= pf_word;
                        poly_pc <= pf_pc;
                        arith_operand_a <= pf_forward_a
                            ? poly_a_sign(v2_exec_result, pf_word) : pf_opa;
                        arith_operand_b <= pf_forward_b
                            ? poly_b_sign(v2_exec_result, pf_word) : pf_opb;
                        v2_exec_op <= poly_exec_op(pf_word.op);
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                        poly_final <= pf_word.final_op;
                        pf_pc <= pf_next_pc;
                        pf_word <= pf_next_word;
                        pf_state <= poly_fetch_behind(pf_word) ? PF_PRED : PF_IDLE;
                    end else begin
                        if (poly_word.op == X87_POLY_OP_CMP) begin
                            if (poly_word.setp == X87_POLY_SETP_SWAP)
                                poly_swap <= v2_compare_less;
                            else if (!v2_compare_less)
                                poly_id <= poly_id + 2'd1;
                        end
                        if (poly_word.op == X87_POLY_OP_RND)
                            poly_quadrant <= v2_exec_result.sign
                                ? 2'd0 - poly_round_low3[1:0]
                                : poly_round_low3[1:0];
                        // Words the fetch skipped do not run.
                        poly_pc <= (pf_state == PF_IDLE) ? poly_pc + 8'd1 : pf_pc;
                        poly_phase <= POLY_DECODE;
                        pf_state <= PF_IDLE;
                    end
                end else begin
                    case (pf_state)
                        PF_PRED: begin
                            if (!x87_poly_pred(pf_word.pred, poly_quadrant, poly_cosine,
                                               poly_swap, poly_xneg, poly_yneg,
                                               poly_id, poly_k0)) begin
                                pf_pc <= pf_next_pc;
                                pf_word <= pf_next_word;
                            end else if ((pf_word.op == X87_POLY_OP_AUX) ||
                                         (pf_word.op == X87_POLY_OP_XTR)) begin
                                pf_state <= PF_STOP;
                            end else begin
                                pf_state <= (pf_word.a_const != pf_word.b_const)
                                            ? PF_CAP1 : PF_CAPA;
                            end
                        end
                        PF_CAP1: begin
                            pf_opa <= poly_a_sign(pf_opa_read, pf_word);
                            pf_opb <= poly_b_sign(pf_opb_read, pf_word);
                            pf_state <= PF_READY;
                        end
                        PF_CAPA: begin
                            pf_opa <= poly_a_sign(pf_opa_read, pf_word);
                            pf_state <= PF_CAPB;
                        end
                        PF_CAPB: begin
                            pf_opb <= poly_b_sign(pf_opb_read, pf_word);
                            pf_state <= PF_READY;
                        end
                        default: ;
                    endcase
                end
            end
            // XTR: A = sig/2^52 * 2^(exp - 16383) once sig[52] is set (a
            // denormal shifts left, bounded); m = sig/2^52, or sig/2^53 and
            // k + 1 when sig >= sqrt(2). x = 2^k exactly: PE only if the
            // final rounding is inexact.
            POLY_XTR_NORM: begin
                if (!xtr_sig[52] && (xtr_exp > -17'sd64)) begin
                    xtr_sig <= {xtr_sig[51:0], xtr_guard};
                    xtr_guard <= xtr_round;
                    xtr_round <= xtr_sticky;
                    xtr_exp <= xtr_exp - 17'sd1;
                end else begin
                    xtr_ge <= xtr_sig >= X87_POLY_SQRT2_SIG;
                    poly_exact <= (xtr_sig == {1'b1, 52'h0}) &&
                                  !xtr_guard && !xtr_round && !xtr_sticky;
                    poly_phase <= POLY_XTR_M;
                end
            end
            POLY_XTR_M: begin                      // m into dest; k + 2^15
                xtr_u <= 16'(xtr_exp + $signed({16'd0, xtr_ge}) + 17'sd16385);
                poly_phase <= POLY_XTR_K;
            end
            POLY_XTR_K: begin                      // 2^27 + 2^15 + k into B's slot
                poly_k0 <= xtr_u == 16'h8000;
                poly_pc <= poly_pc + 8'd1;
                poly_phase <= POLY_DECODE;
            end
            default: ;
        endcase

        if (math_done) begin
            command_complete_pulse <= 1'b1;
            status_flags[9] <= v2_exec_math_done &&
                               (v2_exec_op != X87_ARITH_DIV) &&
                               (v2_exec_op != X87_ARITH_SQRT) &&
                               v2_exec_rounded_up;

            if (v2_exec_math_done && arith_compare) begin
                status_flags[14] <= v2_compare_equal ||
                                    v2_compare_unordered; // C3
                status_flags[10] <= v2_compare_unordered; // C2
                status_flags[8] <= v2_compare_less ||
                                   v2_compare_unordered;  // C0
            end

            if (trans_done)
                status_flags[10] <= v2_exec_range_incomplete; // C2

            if (math_invalid)
                raise_invalid();

            if (math_denormal_operand) begin
                status_flags[1] <= 1'b1; // DE
                if (!control_word[1]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_divide_by_zero) begin
                status_flags[2] <= 1'b1; // ZE
                if (!control_word[2]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_overflow) begin
                status_flags[3] <= 1'b1; // OE
                if (!control_word[3]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_underflow) begin
                status_flags[4] <= 1'b1; // UE
                if (!control_word[4]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            if (math_inexact) begin
                status_flags[5] <= 1'b1; // PE
                if (!control_word[5]) begin
                    status_flags[7] <= 1'b1;
                    status_flags[15] <= 1'b1;
                end
            end

            // A masked invalid operation retires its indefinite/quiet-NaN
            // result and compare pop. An unmasked invalid leaves the stack.
            if (!math_unmasked_exception && !math_range_incomplete) begin
                // The result goes to the stack RAM in the next cycle, which
                // holds PEREQ (stack_write_a) as the result write did. A
                // destination this instruction also pops (FADDP ST(0),ST)
                // is written the cycle after the pop, as it always was, so
                // its tag reads as the result's.
                if (arith_write_result &&
                    (v2_exec_commit == X87_COMMIT_REPLACE_ST0)) begin
                    if (arith_pop_row_written) begin
                        result_write_index <= arith_dest_index;
                        result_write_raw <= x87_to_m80(math_result);
                        result_write_pending <= 1'b1;
                    end else begin
                        write_stack_raw(arith_dest_index, x87_to_m80(math_result));
                    end
                    command_complete_pulse <= 1'b0;
                end
                if (arith_pop_count != 0) begin
                    pop_tags(arith_pop_count);
                    top <= top + arith_pop_count;
                end
            end
        end

        if (core_cmd_valid && core_cmd_ready) begin
            last_fop <= core_cmd_fop;
            command_fop <= core_cmd_fop;
            command_pending <= 1'b1;
            command_held <= !core_cmd_idle;
            masked_cw_load_r <= core_cmd_direct && (direct_data_r[5:0] == 6'h3f);
            stack_addr_a <= accept_top;
            stack_addr_b <= accept_row_b;
            command_row_a <= accept_top;
            command_row_b <= accept_row_b;
            if (core_cmd_direct) begin
                direct_pending <= 1'b0;
                direct_tail_words <= direct_words_r - 2'd1;
                direct_tail_data <= direct_data_r[79:32];
            end
        end

        command_rows_fresh <= (core_cmd_valid && core_cmd_ready) ||
                              ((stack_addr_a == command_row_a) &&
                               (stack_addr_b == command_row_b) &&
                               !stack_write_a && !stack_write_b);

        if (word_in_valid && word_in_ready)
            // Keep PEREQ visible through the CPU microcode branch that
            // observes completion, even though the arithmetic side may drain
            // this queued word independently.
            pereq_release_hold <= 2'b11;

        // A held command whose RAM outputs went stale (the ports moved on
        // without its dispatch) reads its rows again.
        if (command_pending && command_held && held_retired &&
            !command_rows_fresh) begin
            stack_addr_a <= command_row_a;
            stack_addr_b <= command_row_b;
        end

        // Stack operands are synchronous RAM outputs captured from the command
        // acceptance cycle. Execute only after both ports have returned.
        if (command_go) begin
            command_pending <= 1'b0;
            command_held <= 1'b0;
            // Later stages (memory arithmetic's ST0) read these rows again.
            stack_addr_a <= command_row_a;
            stack_addr_b <= command_row_b;
            command_complete_pulse <= 1'b1;
            status_read_pending <= command_decode.status_pending;
            rx_kind <= RX_NONE;
            rx_index <= 5'd0;
            rx_payload <= 80'h0;
            rx_byte_count <= 4'd0;
            tx_kind <= TX_NONE;
            tx_index <= 5'd0;
            tx_byte_offset <= 3'd0;
            tx_generation_done <= 1'b0;

            case (command_decode.action)
                X87_CMD_FNINIT: begin
                    control_word <= 16'h037f;
                    status_flags <= 16'h0000;
                    top <= 3'd0;
                    clear_stack();
                end
                X87_CMD_FNCLEX: begin
                    status_flags[7:0] <= 8'h00;
                    status_flags[15] <= 1'b0;
                end
                X87_CMD_FCHS: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            begin
                                result_write_index <= st0_index;
                                result_write_raw <= x87_to_m80(x87_indefinite());
                                result_write_pending <= 1'b1;
                                command_complete_pulse <= 1'b0;
                            end
                    end else begin
                        write_stack_raw(st0_index, {
                            !stack_read_raw_a[79], stack_read_raw_a[78:0]});
                        command_complete_pulse <= 1'b0;
                    end
                end
                X87_CMD_FABS: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            begin
                                result_write_index <= st0_index;
                                result_write_raw <= x87_to_m80(x87_indefinite());
                                result_write_pending <= 1'b1;
                                command_complete_pulse <= 1'b0;
                            end
                    end else begin
                        write_stack_raw(st0_index, {1'b0, stack_read_raw_a[78:0]});
                        command_complete_pulse <= 1'b0;
                    end
                end
                X87_CMD_FXAM: begin
                    status_flags[9] <= stack_empty(st0_index)
                                     ? 1'b0 : stack_read_data_a.sign;
                    case (stack_empty(st0_index)
                            ? X87_EMPTY : stack_read_data_a.class_id)
                        X87_NAN: begin
                            status_flags[14] <= 1'b0;
                            status_flags[10] <= 1'b0;
                            status_flags[8] <= 1'b1;
                        end
                        X87_NORMAL: begin
                            status_flags[14] <= 1'b0;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b0;
                        end
                        X87_INFINITY: begin
                            status_flags[14] <= 1'b0;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b1;
                        end
                        X87_ZERO: begin
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b0;
                            status_flags[8] <= 1'b0;
                        end
                        X87_EMPTY: begin
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b0;
                            status_flags[8] <= 1'b1;
                        end
                        default: begin                 // Denormal/unsupported
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b0;
                        end
                    endcase
                end
                X87_CMD_PUSH_CONST: begin
                    // Pushed at dispatch; the stack write holds PEREQ.
                    command_complete_pulse <= 1'b0;
                    case (command_decode.argument)
                        4'd0: push_value(x87_one());          // FLD1
                        4'd1: push_const_raw(                 // FLDL2T
                            control_word[11:10] == 2'b10
                                ? 80'h4000_d49a784bcd1b8aff
                                : 80'h4000_d49a784bcd1b8afe);
                        4'd2: push_const_raw(                 // FLDL2E
                            !control_word[10]
                                ? 80'h3fff_b8aa3b295c17f0bc
                                : 80'h3fff_b8aa3b295c17f0bb);
                        4'd3: push_const_raw(                 // FLDPI
                            !control_word[10]
                                ? 80'h4000_c90fdaa22168c235
                                : 80'h4000_c90fdaa22168c234);
                        4'd4: push_const_raw(                 // FLDLG2
                            !control_word[10]
                                ? 80'h3ffd_9a209a84fbcff799
                                : 80'h3ffd_9a209a84fbcff798);
                        4'd5: push_const_raw(                 // FLDLN2
                            !control_word[10]
                                ? 80'h3ffe_b17217f7d1cf79ac
                                : 80'h3ffe_b17217f7d1cf79ab);
                        default: push_value(x87_zero(1'b0));  // FLDZ
                    endcase
                end
                X87_CMD_FSQRT: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        exec_plain <= 1'b1;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= x87_empty();
                        v2_exec_op <= X87_ARITH_SQRT;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FPTAN: begin                   // FPTAN / FSINCOS
                    fptan_sincos <= command_decode.argument[0];
                    fptan_push_value <= command_decode.argument[0]
                                      ? x87_indefinite() : x87_one();
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            fptan_push_after_result <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else if (!stack_empty(top - 3'd1)) begin
                        logic [2:0] new_top;

                        new_top = top - 3'd1;
                        raise_stack_fault(1'b1);
                        if (control_word[0]) begin
                            write_stack(top, x87_indefinite());
                            stack_addr_b <= new_top;
                            stack_write_data_b <= x87_to_m80(x87_indefinite());
                            stack_write_b <= 1'b1;
                            set_tag_b(new_top, x87_tag(x87_indefinite()));
                            top <= new_top;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= x87_empty();
                        trans_cosine <= 1'b0;
                        trans_atan2 <= 1'b0;
                        trans_logexp <= X87_TRANS_NONE;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                        fptan_trans_pending <= 1'b1;
                    end
                end
                X87_CMD_FPATAN: begin
                    if (stack_empty(st0_index) ||
                        stack_empty(top + 3'd1)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= top + 3'd1;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            pop_tags(2'd1);
                            top <= top + 3'd1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd1;
                        arith_dest_index <= top + 3'd1;
                        arith_operand_a <= stack_read_data_b; // Y = ST(1)
                        arith_operand_b <= stack_read_data_a; // X = ST(0)
                        trans_cosine <= 1'b0;
                        trans_atan2 <= 1'b1;
                        trans_logexp <= X87_TRANS_NONE;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FYL2X: begin                   // FYL2X / FYL2XP1
                    if (stack_empty(st0_index) ||
                        stack_empty(top + 3'd1)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            write_stack(top + 3'd1, x87_indefinite());
                            pop_value();
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd1;
                        arith_dest_index <= top + 3'd1;
                        arith_operand_a <= stack_read_data_b; // y = ST(1)
                        arith_operand_b <= stack_read_data_a; // x = ST(0)
                        trans_cosine <= 1'b0;
                        trans_atan2 <= 1'b0;
                        trans_logexp <= command_decode.argument[0]
                                      ? X87_TRANS_FYL2XP1 : X87_TRANS_FYL2X;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_F2XM1: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            write_stack(st0_index, x87_indefinite());
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= stack_read_data_a; // the program's xneg
                        trans_cosine <= 1'b0;
                        trans_atan2 <= 1'b0;
                        trans_logexp <= X87_TRANS_F2XM1;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FSCALE: begin
                    if (stack_empty(st0_index) ||
                        stack_empty(top + 3'd1)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            write_stack(st0_index, x87_indefinite());
                    end else begin
                        // ST0 * 2^trunc(ST1) in the executor (X87_ARITH_SCALE),
                        // retired as arithmetic: flags, C1, unmasked exceptions.
                        command_complete_pulse <= 1'b0;
                        exec_plain <= 1'b1;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        v2_exec_op <= X87_ARITH_SCALE;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= stack_read_data_b;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FPREM: begin
                    if (stack_empty(st0_index) ||
                        stack_empty(top + 3'd1)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            write_stack(st0_index, x87_indefinite());
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b0;
                        arith_pop_count <= 2'd0;
                        fprem_dividend <= stack_read_data_a;
                        fprem_divisor <= stack_read_data_b;
                        fprem_nearest <= command_decode.argument[0]; // FPREM1
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= stack_read_data_b;
                        v2_exec_op <= X87_ARITH_DIV;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                        fprem_phase <= FPREM_DIVIDE;
                    end
                end
                X87_CMD_TRIG: begin                    // FSIN / FCOS
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        arith_operand_a <= stack_read_data_a;
                        arith_operand_b <= x87_empty();
                        trans_cosine <= command_decode.argument[0];
                        trans_atan2 <= 1'b0;
                        trans_logexp <= X87_TRANS_NONE;
                        v2_exec_op <= X87_ARITH_TRANS;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FRNDINT: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            result_write_index <= st0_index;
                            result_write_raw <= x87_to_m80(x87_indefinite());
                            result_write_pending <= 1'b1;
                            command_complete_pulse <= 1'b0;
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        exec_plain <= 1'b1;
                        arith_compare <= 1'b0;
                        arith_write_result <= 1'b1;
                        arith_pop_count <= 2'd0;
                        arith_dest_index <= st0_index;
                        v2_exec_op <= X87_CONVERT_FRNDINT;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        arith_operand_a <= stack_read_data_a;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                    end
                end
                X87_CMD_FXTRACT: begin
                    // ST0 = exponent (as a real) and push the significand
                    // with exponent 3FFFh. Zero: -inf and ZE; inf: +inf. The
                    // exponent of a finite operand goes through the FILD
                    // conversion into the old ST0 slot.
                    logic [2:0] new_top;
                    x87_reg_t fx_exp;
                    x87_reg_t fx_sig;
                    logic fx_write;
                    logic fx_convert;
                    logic [5:0] fx_lead;

                    new_top = top - 3'd1;
                    fx_write = control_word[0];
                    fx_convert = 1'b0;
                    fx_exp = x87_indefinite();
                    fx_sig = x87_indefinite();
                    fx_lead = x87_leading_zeros(stack_read_data_a.sig);
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                    end else if (!stack_empty(new_top)) begin
                        raise_stack_fault(1'b1);
                    end else begin
                        fx_write = 1'b1;
                        fx_sig = stack_read_data_a;
                        case (stack_read_data_a.class_id)
                            X87_ZERO: begin
                                status_flags[2] <= 1'b1;        // ZE
                                if (!control_word[2]) begin
                                    status_flags[7] <= 1'b1;
                                    status_flags[15] <= 1'b1;
                                    fx_write = 1'b0;
                                end
                                fx_exp = x87_infinity(1'b1);
                            end
                            X87_INFINITY: fx_exp = x87_infinity(1'b0);
                            X87_NAN: begin
                                fx_sig.sig[51] = 1'b1;          // quiet
                                fx_exp = fx_sig;
                            end
                            X87_DENORMAL: begin
                                status_flags[1] <= 1'b1;        // DE
                                fx_convert = 1'b1;
                                fx_sig.sig = stack_read_data_a.sig << fx_lead;
                                fx_sig.exp = 15'h3fff;
                                fx_sig.class_id = X87_NORMAL;
                            end
                            default: begin
                                fx_convert = 1'b1;
                                fx_sig.exp = 15'h3fff;
                            end
                        endcase
                    end
                    if (fx_write) begin
                        if (fx_convert) begin
                            command_complete_pulse <= 1'b0;
                            fxtract_exp_pending <= 1'b1;
                            fxtract_exp_index <= top;
                            v2_exec_op <= X87_CONVERT_FILD;
                            v2_exec_owner <= EXEC_LOAD;
                            v2_exec_size <= 2'd0;              // 16-bit integer
                            v2_exec_transfer <= {48'h0,
                                ((stack_read_data_a.class_id == X87_DENORMAL)
                                     ? 16'(-16'sd16382 - $signed({10'b0, fx_lead}))
                                     : 16'($signed({1'b0, stack_read_data_a.exp}) -
                                           16'sd16383))};
                            v2_exec_pending <= 1'b1;
                        end else begin
                            write_stack(top, fx_exp);
                        end
                        stack_addr_b <= new_top;
                        stack_write_data_b <= x87_to_m80(fx_sig);
                        stack_write_b <= 1'b1;
                        set_tag_b(new_top, x87_tag(fx_sig));
                        top <= new_top;
                    end
                end
                X87_CMD_FDECSTP: top <= top - 3'd1;
                X87_CMD_FINCSTP: top <= top + 3'd1;
                X87_CMD_TX_ENV: begin                 // FNSTENV/FSTENV
                    tx_kind <= TX_ENV;
                    tx_generation_done <= 1'b0;
                    command_complete_pulse <= 1'b0;
                end
                X87_CMD_TX_STATE: begin               // FNSAVE/FSAVE
                    tx_kind <= TX_STATE;
                    tx_generation_done <= 1'b0;
                    command_complete_pulse <= 1'b0;
                    tx_state_shift <= pack_state_pair(
                        stack_read_raw_a, stack_read_raw_b, top);
                end
                X87_CMD_RX_ENV: rx_kind <= RX_ENV;     // FLDENV
                X87_CMD_RX_STATE: rx_kind <= RX_STATE; // FRSTOR
                X87_CMD_ARITH: begin
                    // Register arithmetic and comparisons use the synchronous
                    // stack outputs captured with this command.
                    if (stack_empty(st0_index) ||
                        (command_decode.needs_sti &&
                         stack_empty(cmd_st_index))) begin
                        raise_stack_fault(1'b0);
                        if (command_decode.compare) begin
                            status_flags[14] <= 1'b1;
                            status_flags[10] <= 1'b1;
                            status_flags[8] <= 1'b1;
                        end
                        if (control_word[0]) begin
                            if (command_decode.write_result)
                                write_stack(
                                    command_decode.dest_sti
                                        ? cmd_st_index : st0_index,
                                    x87_indefinite());
                            if (command_decode.pop_count != 0) begin
                                pop_tags(command_decode.pop_count);
                                top <= top + command_decode.pop_count;
                            end
                        end
                    end else begin
                        command_complete_pulse <= 1'b0;
                        exec_plain <= 1'b1;
                        v2_exec_op <= command_decode.exec_op;
                        v2_exec_owner <= EXEC_MATH;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= 64'h0;
                        v2_exec_pending <= 1'b1;
                        arith_compare <= command_decode.compare;
                        arith_quiet_compare <= command_decode.quiet_compare;
                        arith_write_result <= command_decode.write_result;
                        arith_pop_count <= command_decode.pop_count;
                        arith_dest_index <= command_decode.dest_sti
                                          ? cmd_st_index : st0_index;

                        if (!command_decode.needs_sti) begin // FTST uses +0.
                            arith_operand_a <= stack_read_data_a;
                            arith_operand_b <= x87_zero(1'b0);
                        end else if (command_decode.reverse_operands) begin
                            arith_operand_a <= stack_read_data_b;
                            arith_operand_b <= stack_read_data_a;
                        end else begin
                            arith_operand_a <= stack_read_data_a;
                            arith_operand_b <= stack_read_data_b;
                        end
                    end
                end

                // Register stack operations.
                X87_CMD_FLD_ST: begin
                    if (stack_empty(cmd_st_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0])
                            push_value(x87_indefinite());
                    end else begin
                        push_raw_tagged(
                            stack_read_raw_b,
                            tag_word[cmd_st_index*2 +: 2]);
                    end
                end
                X87_CMD_FXCH: begin
                    if (stack_empty(st0_index) || stack_empty(cmd_st_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            stack_addr_a <= st0_index;
                            stack_write_data_a <= x87_to_m80(x87_indefinite());
                            stack_write_a <= 1'b1;
                            if (cmd_st_index != st0_index) begin
                                stack_addr_b <= cmd_st_index;
                                stack_write_data_b <= x87_to_m80(x87_indefinite());
                                stack_write_b <= 1'b1;
                            end
                            set_tag_a(st0_index, 2'b10);
                            set_tag_b(cmd_st_index, 2'b10);
                        end
                    end else begin
                        stack_addr_a <= st0_index;
                        stack_write_data_a <= stack_read_raw_b;
                        stack_write_a <= 1'b1;
                        if (cmd_st_index != st0_index) begin
                            stack_addr_b <= cmd_st_index;
                            stack_write_data_b <= stack_read_raw_a;
                            stack_write_b <= 1'b1;
                        end
                        set_tag_a(st0_index, tag_word[cmd_st_index*2 +: 2]);
                        set_tag_b(cmd_st_index, tag_word[st0_index*2 +: 2]);
                    end
                end
                // FFREEP (argument 1) also pops.
                X87_CMD_FFREE: begin
                    if (cmd_st_index != st0_index || !command_decode.argument[0])
                        set_tag_a(cmd_st_index, 2'b11);
                    if (command_decode.argument[0])
                        pop_value();
                end
                // FST ST(i) (argument 1) does not pop.
                X87_CMD_FSTP_ST: begin
                    if (stack_empty(st0_index)) begin
                        raise_stack_fault(1'b0);
                        if (control_word[0]) begin
                            write_stack(cmd_st_index, x87_indefinite());
                            if (!command_decode.argument[0])
                                pop_value();
                        end
                    end else begin
                        write_stack_raw(cmd_st_index, stack_read_raw_a);
                        if (!command_decode.argument[0])
                            pop_value();
                    end
                end

                X87_CMD_MEMORY_MATH: begin
                    memory_math_pending <= 1'b1;
                    memory_math_st0 <= stack_read_data_a;
                    memory_math_mul <= command_decode.exec_op == X87_ARITH_MUL;
                    memory_math_div <= command_decode.exec_op == X87_ARITH_DIV;
                    memory_math_compare <= command_decode.compare;
                    memory_math_subtract <= command_decode.exec_op == X87_ARITH_SUB;
                    memory_math_reverse <= command_decode.reverse_operands;
                    memory_math_pop <= command_decode.pop_count != 0;
                    rx_kind <= rx_kind_t'(command_decode.argument);
                end
                X87_CMD_LOAD:
                    rx_kind <= rx_kind_t'(command_decode.argument);
                X87_CMD_STORE:
                    start_store(
                        command_decode.argument[0],
                        command_decode.argument[2:1],
                        command_decode.argument[3]);
                X87_CMD_STORE_BCD:
                    start_bcd_store();
                default: ;
            endcase
        end

        if ((rx_kind != RX_NONE) && transfer_pop_valid) begin
            rx_payload <= rx_payload_next;
            rx_byte_count <= rx_byte_count_next;
            case (rx_kind)
                RX_CONTROL: begin
                    if (rx_byte_count_next >= 4'd2) begin
                        control_word <= x87_load_control_word(rx_payload_next[15:0]);
                        rx_kind <= RX_NONE;
                    end
                end
                RX_M32: begin
                    if (rx_byte_count_next >= 4'd4) begin
                        if (m32_is_normal(rx_payload_next[31:0]) &&
                            !memory_math_pending) begin
                            // FLD m32 of a normal value pushes now.
                            push_raw_tagged(normal_m32_to_m80(
                                rx_payload_next[31:0]), 2'b00);
                        end else if (m32_is_normal(rx_payload_next[31:0])) begin
                            // Memory arithmetic starts now.
                            launch_memory_math(normal_m32_value(
                                rx_payload_next[31:0]));
                        end else begin
                            v2_exec_op <= X87_CONVERT_FLD_M32;
                            v2_exec_owner <= EXEC_LOAD;
                            v2_exec_size <= 2'd0;
                            v2_exec_transfer <= {32'h0,
                                                 rx_payload_next[31:0]};
                            v2_exec_pending <= 1'b1;
                        end
                        rx_kind <= RX_NONE;
                    end
                end
                RX_M64: begin
                    if (rx_byte_count_next >= 4'd8) begin
                        if (m64_is_normal(rx_payload_next[63:0]) &&
                            !memory_math_pending) begin
                            // FLD m64 of a normal value pushes now.
                            push_raw_tagged(normal_m64_to_m80(
                                rx_payload_next[63:0]), 2'b00);
                        end else if (m64_is_normal(rx_payload_next[63:0])) begin
                            launch_memory_math(normal_m64_value(
                                rx_payload_next[63:0]));
                        end else begin
                            v2_exec_op <= X87_CONVERT_FLD_M64;
                            v2_exec_owner <= EXEC_LOAD;
                            v2_exec_size <= 2'd0;
                            v2_exec_transfer <= rx_payload_next[63:0];
                            v2_exec_pending <= 1'b1;
                        end
                        rx_kind <= RX_NONE;
                    end
                end
                RX_BCD: begin
                    if (rx_byte_count_next >= 4'd10) begin
                        tx_state_shift <= {rx_payload_next[71:0], 64'h0};
                        bcd_shift_count <= 7'd64;
                        bcd_load_negate <= rx_payload_next[79];
                        bcd_load_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_M80: begin
                    if (rx_byte_count_next >= 4'd10) begin
                        if (memory_math_pending) begin
                            push_pending_raw <= rx_payload_next;
                            push_pending_tag <= stack_tag_from_m80(rx_payload_next);
                            push_pending <= 1'b1;
                        end else begin
                            // FLD m80 pushes now.
                            push_raw_tagged(rx_payload_next,
                                            stack_tag_from_m80(rx_payload_next));
                        end
                        rx_kind <= RX_NONE;
                    end
                end
                RX_I16: begin
                    if (rx_byte_count_next >= 4'd2) begin
                        v2_exec_op <= X87_CONVERT_FILD;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd0;
                        v2_exec_transfer <= {48'h0, rx_payload_next[15:0]};
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_I32: begin
                    if (rx_byte_count_next >= 4'd4) begin
                        v2_exec_op <= X87_CONVERT_FILD;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd1;
                        v2_exec_transfer <= {32'h0, rx_payload_next[31:0]};
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_I64: begin
                    if (rx_byte_count_next >= 4'd8) begin
                        v2_exec_op <= X87_CONVERT_FILD;
                        v2_exec_owner <= EXEC_LOAD;
                        v2_exec_size <= 2'd2;
                        v2_exec_transfer <= rx_payload_next[63:0];
                        v2_exec_pending <= 1'b1;
                        rx_kind <= RX_NONE;
                    end
                end
                RX_ENV, RX_STATE: begin
                    if (rx_index < 5'd7) begin
                        accept_environment_word(
                            rx_index[2:0], transfer_pop_data[15:0]);
                        if ((rx_kind == RX_ENV) && (rx_index == 5'd6))
                            rx_kind <= RX_NONE;
                        else
                            rx_index <= rx_index + 5'd1;
                    end else begin
                        case (rx_index)
                            5'd7, 5'd12, 5'd17, 5'd22:
                                rx_state_shift[31:0] <=
                                    transfer_pop_data[31:0];
                            5'd8, 5'd13, 5'd18, 5'd23:
                                rx_state_shift[63:32] <=
                                    transfer_pop_data[31:0];
                            5'd9, 5'd14, 5'd19, 5'd24:
                                rx_state_shift[95:64] <=
                                    transfer_pop_data[31:0];
                            5'd10, 5'd15, 5'd20, 5'd25:
                                rx_state_shift[127:96] <=
                                    transfer_pop_data[31:0];
                            5'd11: commit_state_pair(
                                2'd0, transfer_pop_data[31:0]);
                            5'd16: commit_state_pair(
                                2'd1, transfer_pop_data[31:0]);
                            5'd21: commit_state_pair(
                                2'd2, transfer_pop_data[31:0]);
                            5'd26: begin
                                commit_state_pair(
                                    2'd3, transfer_pop_data[31:0]);
                                rx_kind <= RX_NONE;
                            end
                            default: ;
                        endcase
                        if (rx_index != 5'd26)
                            rx_index <= rx_index + 5'd1;
                    end
                end
                default: ;
            endcase
        end

        // Output generation runs ahead until the shared queue fills. State
        // stream sequencing advances when a word enters the queue, not when
        // the CPU eventually reads it.
        if (tx_produce_fire) begin
            if (tx_kind == TX_STATE) begin
                // Register pairs are packed at indices 6, 11, 16 and 21 from
                // rows read two entries ahead (TOP+0/1, +2/3, +4/5, +6/7).
                case (tx_index)
                    5'd6, 5'd11, 5'd16, 5'd21:
                        tx_state_shift <= pack_state_pair(
                            stack_read_raw_a, stack_read_raw_b, fsave_pair_row);
                    default:
                        if (tx_index >= 5'd7)
                            tx_state_shift <= tx_state_shift >> 32;
                endcase
                case (tx_index)
                    5'd9: begin
                        stack_addr_a <= top + 3'd2;
                        stack_addr_b <= top + 3'd3;
                    end
                    5'd14: begin
                        stack_addr_a <= top + 3'd4;
                        stack_addr_b <= top + 3'd5;
                    end
                    5'd19: begin
                        stack_addr_a <= top + 3'd6;
                        stack_addr_b <= top + 3'd7;
                    end
                    default: ;
                endcase
            end else if (tx_kind == TX_VALUE) begin
                tx_state_shift <= tx_state_shift >> 32;
            end

            if (((tx_kind == TX_VALUE) &&
                 (tx_index + 5'd1 == {3'h0, tx_count})) ||
                ((tx_kind == TX_ENV) && (tx_index == 5'd6)) ||
                ((tx_kind == TX_STATE) && (tx_index == 5'd26))) begin
                tx_generation_done <= 1'b1;
            end else begin
                tx_index <= tx_index + 5'd1;
            end
        end

        if (read_req_valid && read_req_ready) begin
            pereq_release_hold <= 2'b11;
            if (!read_req_data_port) begin
                // FNSTSW has register and memory forms. Other f8
                // miscellaneous reads return the control word; environment
                // streams use fc instead.
                read_resp_data <= fop_reads_status(last_fop)
                                ? {16'h0, status_word}
                                : {16'h0, control_word};
                status_read_pending <= 1'b0;
            end else begin
                read_resp_data <= select_transfer_bytes(
                    transfer_pop_data[31:0], tx_byte_offset, read_req_be);
                if (tx_entry_consumed)
                    tx_byte_offset <= 3'd0;
                else
                    tx_byte_offset <= tx_byte_offset + tx_request_bytes;
            end
            read_resp_valid <= 1'b1;
        end

        // Architectural save side effects occur only after the CPU consumes
        // the final queued word, preserving the previous visible ordering.
        if (tx_consume_fire && tx_generation_done &&
            (transfer_count == 2'd1)) begin
            if (tx_kind == TX_ENV)
                control_word[5:0] <= 6'h3f;
            else if (tx_kind == TX_STATE) begin
                control_word <= 16'h037f;
                status_flags <= 16'h0000;
                top <= 3'd0;
                clear_stack();
            end
            tx_kind <= TX_NONE;
            tx_index <= 5'd0;
            tx_generation_done <= 1'b0;
            tx_byte_offset <= 3'd0;
        end
    end

    // Apply this cycle's tag requests in architectural order.
    begin
        logic [15:0] tag_next;
        tag_next = tag_full_we ? tag_full_value : tag_word;
        if (tag_a_we)
            tag_next[tag_a_index*2 +: 2] = tag_a_value;
        if (tag_b_we)
            tag_next[tag_b_index*2 +: 2] = tag_b_value;
        if (tag_pop_count != 0)
            tag_next[top*2 +: 2] = 2'b11;
        if (tag_pop_count == 2)
            tag_next[(top + 3'd1)*2 +: 2] = 2'b11;
        tag_word <= tag_next;
    end
end

`ifdef Z486_COSIM
`ifndef Z486_USE_ALTERA_MEMORY
// Co-simulation snapshot restore: the architectural x87 state of the 86Box
// reference. Rows are physical, as the stack RAM and tag word hold them.
export "DPI-C" task cosim_load_x87;
task cosim_load_x87(input int cw, input int sw, input int tw, input int row,
                    input int lo, input int hi, input int se);
    if (row < 0) begin
        control_word = cw[15:0];
        status_flags = sw[15:0];
        top = sw[13:11];
        tag_word = tw[15:0];
    end else begin
        stack_mem.mem[row[2:0]] = {se[15:0], hi, lo};
    end
endtask

// The same state for the co-simulation's checks: 0 control, 1 status (TOP in
// 13:11), 2 tag, then each physical row's significand low, high, sign/exponent.
export "DPI-C" function cosim_x87_word;
function int cosim_x87_word(input int idx);
    logic [79:0] r;
    if (idx == 0) return int'(control_word);
    if (idx == 1) return int'(status_word);
    if (idx == 2) return int'(architectural_tag_word());
    r = stack_mem.mem[3'((idx - 3) / 3)];
    case ((idx - 3) % 3)
        0: return int'(r[31:0]);
        1: return int'(r[63:32]);
        default: return int'(r[79:64]);
    endcase
endfunction
`endif
`endif

endmodule
