// x87 timing monitor (measurement only; tb_protected_mode.sv includes it with
// X87_TIMING_MON defined, as build_timing.sh does).
//
// Port 0xD0 is a marker port: a 32-bit OUT of {1'b1, 23'h0, id} opens window
// <id>, an OUT of 0 closes the open window. Every marker prints
//   XMARK <open|close> <id> <cycle>
// on stdout. With +x87log=<file>, every cycle inside a window is written to
// <file> as one line of hex fields (see the $fwrite below for the layout).
generate
if (ENABLE_X87) begin : gen_x87_timing_mon
    integer xm_fd = 0;
    logic   xm_open = 1'b0;
    logic [7:0] xm_id = 8'h0;
    string  xm_file;

    initial begin
        if ($value$plusargs("x87log=%s", xm_file))
            xm_fd = $fopen(xm_file, "w");
    end
    final begin
        if (xm_fd != 0) $fclose(xm_fd);
    end

    // Same acceptance condition as the testbench's write path.
    wire xm_marker = reset_n && valid && ready && !rd_busy && write && io && !inta &&
                     ({addr[15:2], 2'b00} == 16'h00D0);

    // CPU-side flags
    wire [15:0] xm_cpu = {
        dut.i_issue,                 // 15
        dut.i_first,                 // 14
        dut.uc_active,               // 13
        dut.uc_exec,                 // 12
        dut.stall_mem,               // 11
        dut.stall_wio,               // 10
        dut.stall_x87_direct,        // 9
        dut.stall,                   // 8
        dut.x87_req_selected,        // 7  paging presents an x87 port cycle
        dut.x87_req_accepted,        // 6
        dut.dcache_req_write,        // 5
        dut.dcache_req_phys_addr_raw[2], // 4 (1 = FC data port)
        dut.mem_servicing,           // 3
        dut.pb_valid,                // 2
        dut.d2_ready,                // 1
        dut.throttle_parked_r        // 0
    };
    // x87_unit / bridge flags
    wire [15:0] xm_unit = {
        dut.x87.direct_active,       // 15
        dut.x87.direct_issued_r,     // 14
        dut.x87.direct_data_valid_r, // 13
        dut.x87.direct_ready,        // 12
        dut.x87.direct_reg_r,        // 11
        dut.x87.dcmd_pending_r,      // 10
        dut.x87.direct_st_r,         // 9
        dut.x87.posted_pending,      // 8
        dut.x87.br_accepted,         // 7
        dut.x87.br_read_complete,    // 6
        dut.x87.gen_x87.bridge.write_r,      // 5
        dut.x87.gen_x87.bridge.data_port_r,  // 4
        2'(dut.x87.st_state),        // 3:2
        2'(dut.x87.gen_x87.bridge.state)     // 1:0
    };
    // x87_control flags
    wire [31:0] xm_ctl = {
        dut.x87.gen_x87.control.cmd_valid,           // 31
        dut.x87.gen_x87.control.cmd_ready,           // 30
        dut.x87.gen_x87.control.core_cmd_ready,      // 29
        dut.x87.gen_x87.control.busy_n,              // 28
        dut.x87.gen_x87.control.pereq,               // 27
        dut.x87.gen_x87.control.command_pending,     // 26
        dut.x87.gen_x87.control.direct_pending,  // 25
        dut.x87.gen_x87.control.v2_exec_pending,     // 24
        dut.x87.gen_x87.control.v2_exec_busy,        // 23
        dut.x87.gen_x87.control.v2_exec_done,        // 22
        dut.x87.gen_x87.control.push_pending,        // 21
        dut.x87.gen_x87.control.result_write_pending,// 20
        dut.x87.gen_x87.control.memory_math_pending, // 19
        dut.x87.gen_x87.control.store_pending,       // 18
        dut.x87.gen_x87.control.pop_pending,         // 17
        dut.x87.gen_x87.control.stack_write_a ||
        dut.x87.gen_x87.control.stack_write_b,       // 16
        dut.x87.gen_x87.control.word_in_valid &&
        dut.x87.gen_x87.control.word_in_ready,       // 15
        dut.x87.gen_x87.control.read_req_valid &&
        dut.x87.gen_x87.control.read_req_ready,      // 14
        dut.x87.gen_x87.control.read_resp_valid,     // 13
        dut.x87.gen_x87.control.status_read_pending, // 12
        dut.x87.gen_x87.control.command_complete_pulse, // 11
        (dut.x87.gen_x87.control.pereq_release_hold != 2'b00), // 10
        (dut.x87.gen_x87.control.poly_phase != 0),   // 9
        (dut.x87.gen_x87.control.fprem_phase != 0),  // 8
        (dut.x87.gen_x87.control.fscale_phase != 0), // 7
        (dut.x87.gen_x87.control.logexp_phase != 0), // 6
        dut.x87.gen_x87.control.fptan_trans_pending ||
        dut.x87.gen_x87.control.fptan_div_pending ||
        dut.x87.gen_x87.control.fptan_push_after_result, // 5
        dut.x87.gen_x87.control.bcd_convert_pending ||
        dut.x87.gen_x87.control.bcd_load_pending,    // 4
        2'(dut.x87.gen_x87.control.transfer_count),  // 3:2
        2'(dut.x87.gen_x87.control.v2_exec_owner)    // 1:0
    };
    wire [7:0] xm_kinds = {4'(dut.x87.gen_x87.control.rx_kind),
                           2'(dut.x87.gen_x87.control.tx_kind),
                           2'b00};

    always @(posedge clk) begin
        if (xm_marker) begin
            if (dout[31]) begin
                xm_open <= 1'b1;
                xm_id   <= dout[7:0];
                $display("XMARK open %0d %0d", dout[7:0], cycle);
                if (xm_fd != 0) $fwrite(xm_fd, "O %0d %0d\n", dout[7:0], cycle);
            end else begin
                xm_open <= 1'b0;
                $display("XMARK close %0d %0d sw=%04x tags=%04x", xm_id, cycle,
                         {dut.x87.gen_x87.control.status_flags[15:14],
                          dut.x87.gen_x87.control.top,
                          dut.x87.gen_x87.control.status_flags[10:0]},
                         dut.x87.gen_x87.control.tag_word);
                if (xm_fd != 0) $fwrite(xm_fd, "X %0d %0d\n", xm_id, cycle);
            end
        end
        if (xm_open && (xm_fd != 0)) begin
            // cycle eip opcode modrm cpu unit ctl kinds uc_addr exec_op exec_uaddr
            // last_fop {word_in_be, read_req_be}
            $fwrite(xm_fd, "%0d %h %h %h %h %h %h %h %h %h %h %h %h\n",
                    cycle, dut.EIP, dut.i_bus.opcode, dut.i_bus.modrm,
                    xm_cpu, xm_unit, xm_ctl, xm_kinds, dut.uc_addr,
                    8'(dut.x87.gen_x87.control.v2_exec_op),
                    dut.x87.gen_x87.control.v2_exec_uaddr,
                    dut.x87.gen_x87.control.last_fop,
                    {dut.x87.gen_x87.control.word_in_be,
                     dut.x87.gen_x87.control.read_req_be});
        end
    end
end
endgenerate
