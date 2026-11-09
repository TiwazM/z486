//
// Bus Interface Unit
// External bus arbitration, cycle types and response tracking for the caches, I/O and x87
//
module bus_unit
    import z486_pkg::*;
#(
    parameter ENABLE_X87 = 0
)
(
    // Clock and reset
    input  logic clk,
    input  logic reset_n,
    input  logic x87_off,               // Dev menu: no coprocessor

    // Paging unit: demand request, cycle type and completion
    input  logic dcache_req_valid,
    input  logic dcache_req_write,
    input  logic [3:0] dcache_req_be,
    input  logic [31:0] dcache_direct_wdata,
    input  logic dcache_req_is_io,
    input  logic dcache_req_is_inta,
    input  logic dcache_req_is_vga_mem,
    input  logic dcache_req_is_x87,
    output logic dcache_req_accepted,
    output logic dcache_req_complete,
    output logic dcache_read_complete,
    output logic [31:0] dcache_rdata,

    // Execution core: WR_FAST store in flight
    input  logic fast_store_valid,

    // FPU: coprocessor data-port cycles
    output logic x87_req_selected,
    input  logic x87_req_accepted,
    input  logic x87_req_complete,
    input  logic x87_read_complete,
    input  logic [31:0] x87_rdata,

    // Cache unit: request routing and D-cache CPU response
    input  logic normal_cache_req,
    input  logic dcache_req_is_device_mmio,
    input  logic [31:0] dcache_req_phys_addr,
    input  logic dcache_cpu_ready,
    input  logic dcache_cpu_wr_ready,
    input  logic dcache_cpu_resp_valid,
    input  logic [31:0] dcache_cpu_dout,
    input  logic dcache_stores_drained,

    // Cache unit: line fills and write-buffer drain
    input  logic dcache_mem_valid,
    input  logic [31:0] dcache_mem_addr,
    input  logic dcache_mem_write,
    input  logic [3:0] dcache_mem_be,
    input  logic [7:0] dcache_mem_burstcount,
    input  logic [31:0] dcache_mem_din,
    output logic dcache_mem_ready,
    output logic dcache_mem_resp_valid,
    output logic dcache_mem_line_resp_valid,
    input  logic icache_mem_valid,
    input  logic [31:0] icache_mem_addr,
    input  logic [3:0] icache_mem_be,
    input  logic [7:0] icache_mem_burstcount,
    output logic icache_mem_ready,
    output logic icache_mem_resp_valid,
    output logic icache_mem_line_resp_valid,
    output logic ext_valid_r,
    output logic direct_rd_pending,
    output logic dcache_read_pending,
    output logic icache_read_pending,

    // External bus (XA/XD)
    output logic valid,
    input  logic ready,
    output logic [31:2] addr,
    output logic [3:0] be,
    output logic [7:0] burstcount,
    output logic line_read,
    output logic write,
    output logic io,
    output logic inta,
    output logic [31:0] dout,
    input  logic [31:0] din,
    input  logic resp_valid,
    input  logic line_resp_valid
);


logic [7:0] dcache_rd_pending;
logic [7:0] icache_rd_pending;
// direct_rd_pending: port

assign x87_req_selected = ENABLE_X87 && !x87_off && dcache_req_valid && dcache_req_is_x87;
// With the x87 off a 486SX has no coprocessor and no coprocessor bus cycles:
// a port cycle that an ESC routine still issues completes here, writes
// dropped and reads all ones.
wire x87_sink = ENABLE_X87 && x87_off && dcache_req_valid && dcache_req_is_x87;

wire dcache_direct_req = dcache_req_valid && !x87_req_selected && !x87_sink &&
                         (dcache_req_is_io || dcache_req_is_inta ||
                          dcache_req_is_vga_mem ||
                          dcache_req_is_device_mmio);
assign dcache_read_pending = (dcache_rd_pending != 8'd0);
assign icache_read_pending = (icache_rd_pending != 8'd0);
// I/O and INTA transactions must not overtake older posted stores.  This is
// what makes a CPU-filled Sound Blaster buffer visible before the following
// DSP command lets DMA consume it.  VGA memory already bypasses the posted
// queue, so direct VGA accesses remain mutually ordered without draining
// unrelated normal-RAM stores first.
// A device MMIO window above 128 MB (zSST's BAR) is a device page too; it
// keeps waiting for the posted stores as before.
wire direct_req_ordered = (dcache_req_is_vga_mem && !dcache_req_is_device_mmio) ||
                          dcache_stores_drained;
wire ext_direct_req = dcache_direct_req && direct_req_ordered &&
                      !direct_rd_pending &&
                      !dcache_read_pending && !icache_read_pending;
wire ext_dcache_req = dcache_mem_valid && !ext_direct_req &&
                      !direct_rd_pending && !icache_read_pending;
wire ext_icache_req = icache_mem_valid && !ext_direct_req && !ext_dcache_req &&
                      !direct_rd_pending && !dcache_read_pending;

localparam [1:0] EXT_SRC_NONE   = 2'd0;
localparam [1:0] EXT_SRC_DIRECT = 2'd1;
localparam [1:0] EXT_SRC_DCACHE = 2'd2;
localparam [1:0] EXT_SRC_ICACHE = 2'd3;

// ext_valid_r: port
logic [1:0]  ext_src_r;
logic [31:2] ext_addr_r;
logic [3:0]  ext_be_r;
logic [7:0]  ext_burstcount_r;
logic [31:0] ext_direct_dout_r;
logic [31:0] ext_dcache_dout_r;
logic        ext_write_r;
logic        ext_io_r;
logic        ext_inta_r;

wire ext_direct_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_DIRECT);
wire ext_dcache_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_DCACHE);
wire ext_icache_accept = ext_valid_r && ready && (ext_src_r == EXT_SRC_ICACHE);
wire direct_rd_resp_now = resp_valid &&
                          (direct_rd_pending || (ext_direct_accept && !ext_write_r));

// The L1 takes a write in more cycles than a read (behind another store's
// lookup), so each request type sees its own registered ready.
wire dcache_req_ready = dcache_req_write ? dcache_cpu_wr_ready : dcache_cpu_ready;
wire normal_req_accepted = normal_cache_req ? (dcache_req_ready && !fast_store_valid)
                                            : ext_direct_accept;
wire normal_req_complete = dcache_cpu_resp_valid ||
                           (normal_cache_req && dcache_req_write &&
                            dcache_cpu_wr_ready && !fast_store_valid) ||
                           direct_rd_resp_now ||
                           (ext_direct_accept && ext_write_r);
wire normal_read_complete = dcache_cpu_resp_valid || direct_rd_resp_now;
wire [31:0] normal_rdata = dcache_cpu_resp_valid ? dcache_cpu_dout : din;

assign dcache_req_accepted = x87_sink || (x87_req_selected ? x87_req_accepted : normal_req_accepted);
assign dcache_req_complete = normal_req_complete || x87_req_complete || x87_sink;
assign dcache_read_complete = normal_read_complete || x87_read_complete ||
                              (x87_sink && !dcache_req_write);
assign dcache_rdata = x87_sink ? 32'hFFFF_FFFF :
                      x87_read_complete ? x87_rdata : normal_rdata;

assign addr       = ext_addr_r;
assign be         = ext_be_r;
assign burstcount = ext_burstcount_r;
assign line_read  = ext_valid_r && !ext_write_r &&
                    (ext_burstcount_r == 8'd4) &&
                    ((ext_src_r == EXT_SRC_DCACHE) ||
                     (ext_src_r == EXT_SRC_ICACHE));
assign dout       = (ext_src_r == EXT_SRC_DIRECT) ? ext_direct_dout_r :
                    (ext_src_r == EXT_SRC_DCACHE) ? ext_dcache_dout_r : 32'd0;
assign valid      = ext_valid_r;
assign write      = ext_valid_r && ext_write_r;
assign io         = ext_valid_r && ext_io_r;
assign inta       = ext_valid_r && ext_inta_r;

assign dcache_mem_ready = ext_dcache_accept;
assign icache_mem_ready = ext_icache_accept;
assign dcache_mem_resp_valid = dcache_read_pending && resp_valid;
assign icache_mem_resp_valid = icache_read_pending && resp_valid;
assign dcache_mem_line_resp_valid = dcache_read_pending && line_resp_valid;
assign icache_mem_line_resp_valid = icache_read_pending && line_resp_valid;

always_ff @(posedge clk) begin
    if (!reset_n) begin
        ext_valid_r <= 1'b0;
        ext_src_r <= EXT_SRC_NONE;
        ext_addr_r <= 30'h0;
        ext_be_r <= 4'h0;
        ext_burstcount_r <= 8'h0;
        ext_direct_dout_r <= 32'h0;
        ext_dcache_dout_r <= 32'h0;
        ext_write_r <= 1'b0;
        ext_io_r <= 1'b0;
        ext_inta_r <= 1'b0;
        dcache_rd_pending <= 8'd0;
        icache_rd_pending <= 8'd0;
        direct_rd_pending <= 1'b0;
    end else begin
        // Data is speculative until ext_src_r selects its registered owner.
        // Capturing both sources unconditionally prevents request arbitration
        // and live translation from becoming clock-enable muxes on dout.
        ext_direct_dout_r <= dcache_direct_wdata;
        ext_dcache_dout_r <= dcache_mem_din;

        if (ext_valid_r) begin
            if (ready) begin
                ext_valid_r <= 1'b0;
                ext_src_r <= EXT_SRC_NONE;
            end
        end else if (ext_direct_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_DIRECT;
            ext_addr_r <= dcache_req_phys_addr[31:2];
            ext_be_r <= dcache_req_be;
            ext_burstcount_r <= 8'd1;
            ext_write_r <= dcache_req_write;
            ext_io_r <= dcache_req_is_io;
            ext_inta_r <= dcache_req_is_inta;
        end else if (ext_dcache_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_DCACHE;
            ext_addr_r <= dcache_mem_addr[31:2];
            ext_be_r <= dcache_mem_be;
            ext_burstcount_r <= dcache_mem_burstcount;
            ext_write_r <= dcache_mem_write;
            ext_io_r <= 1'b0;
            ext_inta_r <= 1'b0;
        end else if (ext_icache_req) begin
            ext_valid_r <= 1'b1;
            ext_src_r <= EXT_SRC_ICACHE;
            ext_addr_r <= icache_mem_addr[31:2];
            ext_be_r <= icache_mem_be;
            ext_burstcount_r <= icache_mem_burstcount;
            ext_write_r <= 1'b0;
            ext_io_r <= 1'b0;
            ext_inta_r <= 1'b0;
        end

        if (ext_dcache_accept && !dcache_mem_write)
            dcache_rd_pending <= dcache_mem_burstcount;
        else if (dcache_read_pending && line_resp_valid)
            dcache_rd_pending <= 8'd0;
        else if (dcache_read_pending && resp_valid)
            dcache_rd_pending <= dcache_rd_pending - 8'd1;

        if (ext_icache_accept)
            icache_rd_pending <= icache_mem_burstcount;
        else if (icache_read_pending && line_resp_valid)
            icache_rd_pending <= 8'd0;
        else if (icache_read_pending && resp_valid)
            icache_rd_pending <= icache_rd_pending - 8'd1;

        if (ext_direct_accept && !ext_write_r && !resp_valid)
            direct_rd_pending <= 1'b1;
        else if (direct_rd_pending && resp_valid)
            direct_rd_pending <= 1'b0;
    end
end



endmodule
