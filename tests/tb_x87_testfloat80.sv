`timescale 1ns/1ns

// Runs Berkeley TestFloat vectors through x87_executor in the x87's own
// formats and writes the executor's results in TestFloat's format, for
// testfloat_ver to judge (run_x87_testfloat80.sh): extF80 arithmetic at a
// precision control, FRNDINT, compares, and the conversions FLD/FILD/FST/
// FIST perform. Operands enter as x87_from_m80 makes them, as the register
// stack hands them to the executor.
//
//   +FUNC=<TestFloat function> +PC=<0|2|3> +ROUND=<0..3, the x87 RC>
//   +IN=<testfloat_gen output> +OUT=<file for testfloat_ver>
module tb_x87_testfloat80;

import x87_pkg::*;
import x87_ucode_pkg::*;

logic clk = 1'b0;
logic reset = 1'b1;
logic start = 1'b0;
x87_exec_op_t exec_op;
logic [1:0] integer_size;
logic [1:0] precision_control;
logic [1:0] rounding_mode;
logic quiet_compare;
x87_reg_t operand;
x87_reg_t operand_b;
logic [63:0] transfer_in;
logic busy;
logic done;
x87_reg_t result;
logic invalid;
logic inexact;
logic divide_by_zero;
logic overflow;
logic underflow;
logic [63:0] transfer_out;
logic compare_unordered;
logic compare_less;
logic compare_equal;

always #5 clk = ~clk;

x87_executor dut (
    .clk(clk),
    .reset(reset),
    .start(start),
    .exec_op(exec_op),
    .integer_size(integer_size),
    .precision_control(precision_control),
    .rounding_mode(rounding_mode),
    .quiet_compare(quiet_compare),
    .trans_cosine(1'b0),
    .trans_atan2(1'b0),
    .trans_logexp(2'b00),
    .range_masks(2'b11),
    .operand(operand),
    .operand_b(operand_b),
    .transfer_in(transfer_in),
    .busy(busy),
    .done(done),
    .commit_action(),
    .result(result),
    .transfer_out(transfer_out),
    .invalid(invalid),
    .inexact(inexact),
    .divide_by_zero(divide_by_zero),
    .overflow(overflow),
    .underflow(underflow),
    .denormal_operand(),
    .range_incomplete(),
    .rounded_up(),
    .compare_unordered(compare_unordered),
    .compare_less(compare_less),
    .compare_equal(compare_equal)
);

// The line layouts of the functions (operands, result, flags).
typedef enum {
    K_BIN80,    // extF80 op extF80 -> extF80
    K_UN80,     // extF80 -> extF80
    K_CMP80,    // extF80 rel extF80 -> bool
    K_TO32,     // extF80 -> f32 or i32
    K_TO64,     // extF80 -> f64 or i64
    K_FROM32,   // f32 or i32 -> extF80
    K_FROM64    // f64 or i64 -> extF80
} kind_t;

// TestFloat's flag byte: invalid, infinite (divide by zero), overflow,
// underflow, inexact.
function automatic logic [7:0] flags();
    return {3'b000, invalid, divide_by_zero, overflow, underflow, inexact};
endfunction

task automatic run_op(input integer number);
    integer cycles;
    begin
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        cycles = 0;
        while (!done && (cycles < 1024)) begin
            @(posedge clk);
            cycles = cycles + 1;
        end
        if (!done)
            $fatal(1, "TestFloat case %0d timed out", number);
        @(negedge clk);
    end
endtask

initial begin
    string func, in_path, out_path;
    integer pc, round, fi, fo, fields, cases;
    kind_t kind;
    logic rel_less, rel_equal;      // the relation's terms (le = both)
    logic [79:0] a80, b80, r80;
    logic [63:0] a64, r64;
    logic [31:0] a32, r32;
    logic        r1;
    logic [7:0]  f;

    if (!$value$plusargs("FUNC=%s", func)) $fatal(1, "missing +FUNC");
    if (!$value$plusargs("IN=%s", in_path)) $fatal(1, "missing +IN");
    if (!$value$plusargs("OUT=%s", out_path)) $fatal(1, "missing +OUT");
    if (!$value$plusargs("PC=%d", pc)) pc = 3;
    if (!$value$plusargs("ROUND=%d", round)) round = 0;

    precision_control = pc[1:0];
    rounding_mode = round[1:0];
    quiet_compare = 1'b0;
    integer_size = 2'd2;
    rel_less = 1'b0;
    rel_equal = 1'b0;
    case (func)
        "extF80_add":  begin kind = K_BIN80; exec_op = X87_ARITH_ADD; end
        "extF80_sub":  begin kind = K_BIN80; exec_op = X87_ARITH_SUB; end
        "extF80_mul":  begin kind = K_BIN80; exec_op = X87_ARITH_MUL; end
        "extF80_div":  begin kind = K_BIN80; exec_op = X87_ARITH_DIV; end
        "extF80_sqrt": begin kind = K_UN80;  exec_op = X87_ARITH_SQRT; end
        "extF80_roundToInt": begin kind = K_UN80; exec_op = X87_CONVERT_FRNDINT; end
        // FCOM is the signaling compare, FUCOM the quiet one.
        "extF80_eq":           begin kind = K_CMP80; exec_op = X87_ARITH_COMPARE; quiet_compare = 1'b1; rel_equal = 1'b1; end
        "extF80_le":           begin kind = K_CMP80; exec_op = X87_ARITH_COMPARE; rel_less = 1'b1; rel_equal = 1'b1; end
        "extF80_lt":           begin kind = K_CMP80; exec_op = X87_ARITH_COMPARE; rel_less = 1'b1; end
        "extF80_eq_signaling": begin kind = K_CMP80; exec_op = X87_ARITH_COMPARE; rel_equal = 1'b1; end
        "extF80_le_quiet":     begin kind = K_CMP80; exec_op = X87_ARITH_COMPARE; quiet_compare = 1'b1; rel_less = 1'b1; rel_equal = 1'b1; end
        "extF80_lt_quiet":     begin kind = K_CMP80; exec_op = X87_ARITH_COMPARE; quiet_compare = 1'b1; rel_less = 1'b1; end
        "extF80_to_f32": begin kind = K_TO32; exec_op = X87_CONVERT_FST_M32; end
        "extF80_to_f64": begin kind = K_TO64; exec_op = X87_CONVERT_FST_M64; end
        "extF80_to_i32": begin kind = K_TO32; exec_op = X87_CONVERT_FIST; integer_size = 2'd1; end
        "extF80_to_i64": begin kind = K_TO64; exec_op = X87_CONVERT_FIST; integer_size = 2'd2; end
        "f32_to_extF80": begin kind = K_FROM32; exec_op = X87_CONVERT_FLD_M32; end
        "f64_to_extF80": begin kind = K_FROM64; exec_op = X87_CONVERT_FLD_M64; end
        "i32_to_extF80": begin kind = K_FROM32; exec_op = X87_CONVERT_FILD; integer_size = 2'd1; end
        "i64_to_extF80": begin kind = K_FROM64; exec_op = X87_CONVERT_FILD; integer_size = 2'd2; end
        default: $fatal(1, "unsupported TestFloat function %s", func);
    endcase

    operand = x87_zero(1'b0);
    operand_b = x87_zero(1'b0);
    transfer_in = 64'h0;
    repeat (4) @(posedge clk);
    reset = 1'b0;
    repeat (2) @(posedge clk);

    fi = $fopen(in_path, "r");
    if (fi == 0) $fatal(1, "cannot open %s", in_path);
    fo = $fopen(out_path, "w");
    if (fo == 0) $fatal(1, "cannot open %s", out_path);

    cases = 0;
    while (!$feof(fi)) begin
        case (kind)
            K_BIN80, K_CMP80: fields = $fscanf(fi, "%h %h %h %h\n", a80, b80, r80, f);
            K_UN80, K_TO32, K_TO64: fields = $fscanf(fi, "%h %h %h\n", a80, r80, f);
            K_FROM32: fields = $fscanf(fi, "%h %h %h\n", a32, r80, f);
            default:  fields = $fscanf(fi, "%h %h %h\n", a64, r80, f);
        endcase
        if (fields < 3) continue;
        cases = cases + 1;

        operand = x87_from_m80(a80);
        operand_b = x87_from_m80(b80);
        case (kind)
            K_FROM32: transfer_in = {32'h0, a32};
            K_FROM64: transfer_in = a64;
            default:  transfer_in = 64'h0;
        endcase
        run_op(cases);

        case (kind)
            K_BIN80:  $fwrite(fo, "%h %h %h %h\n", a80, b80, x87_to_m80(result), flags());
            K_UN80:   $fwrite(fo, "%h %h %h\n", a80, x87_to_m80(result), flags());
            K_CMP80: begin
                r1 = !compare_unordered &&
                     ((rel_less && compare_less) || (rel_equal && compare_equal));
                $fwrite(fo, "%h %h %h %h\n", a80, b80, r1, flags());
            end
            K_TO32: begin
                r32 = transfer_out[31:0];
                $fwrite(fo, "%h %h %h\n", a80, r32, flags());
            end
            K_TO64:   $fwrite(fo, "%h %h %h\n", a80, transfer_out, flags());
            K_FROM32: $fwrite(fo, "%h %h %h\n", a32, x87_to_m80(result), flags());
            default:  $fwrite(fo, "%h %h %h\n", a64, x87_to_m80(result), flags());
        endcase
    end
    $fclose(fi);
    $fclose(fo);
    $display("tb_x87_testfloat80: %s pc=%0d rc=%0d cases=%0d", func, pc, round, cases);
    $finish;
end

endmodule
