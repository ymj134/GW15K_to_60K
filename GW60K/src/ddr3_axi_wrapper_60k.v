// ============================================================================
// 60K DDR3 AXI wrapper
// ----------------------------------------------------------------------------
// Wraps DDR3_Memory_Interface_Top and keeps the long DDR3 IP port list out of
// top.v. AXI width/ID/address widths match the generated DDR3 IP.
// ============================================================================
`timescale 1ns / 1ps

module ddr3_axi_wrapper_60k (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         memory_clk,
    input  wire         pll_lock,
    output wire         pll_stop,
    output wire         clk_out,
    output wire         ddr_rst,
    output wire         init_calib_complete,

    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,
    input  wire [3:0]   s_axi_awid,
    input  wire [28:0]  s_axi_awaddr,
    input  wire [7:0]   s_axi_awlen,
    input  wire [2:0]   s_axi_awsize,
    input  wire [1:0]   s_axi_awburst,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,
    input  wire [255:0] s_axi_wdata,
    input  wire [31:0]  s_axi_wstrb,
    input  wire         s_axi_wlast,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,
    output wire [1:0]   s_axi_bresp,
    output wire [3:0]   s_axi_bid,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,
    input  wire [3:0]   s_axi_arid,
    input  wire [28:0]  s_axi_araddr,
    input  wire [7:0]   s_axi_arlen,
    input  wire [2:0]   s_axi_arsize,
    input  wire [1:0]   s_axi_arburst,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready,
    output wire [255:0] s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire [3:0]   s_axi_rid,
    output wire         s_axi_rlast,

    output wire [13:0]  O_ddr_addr,
    output wire [2:0]   O_ddr_ba,
    output wire         O_ddr_cs_n,
    output wire         O_ddr_ras_n,
    output wire         O_ddr_cas_n,
    output wire         O_ddr_we_n,
    output wire         O_ddr_clk,
    output wire         O_ddr_clk_n,
    output wire         O_ddr_cke,
    output wire         O_ddr_odt,
    output wire         O_ddr_reset_n,
    output wire [3:0]   O_ddr_dqm,
    inout  wire [31:0]  IO_ddr_dq,
    inout  wire [3:0]   IO_ddr_dqs,
    inout  wire [3:0]   IO_ddr_dqs_n
);

wire sr_req  = 1'b0;
wire ref_req = 1'b0;
wire sr_ack;
wire ref_ack;
wire burst   = 1'b0;

DDR3_Memory_Interface_Top u_ddr3 (
    .clk                 (clk),
    .pll_stop            (pll_stop),
    .memory_clk          (memory_clk),
    .pll_lock            (pll_lock),
    .rst_n               (rst_n),
    .clk_out             (clk_out),
    .ddr_rst             (ddr_rst),
    .init_calib_complete (init_calib_complete),

    .s_axi_awvalid       (s_axi_awvalid),
    .s_axi_awready       (s_axi_awready),
    .s_axi_awid          (s_axi_awid),
    .s_axi_awaddr        (s_axi_awaddr),
    .s_axi_awlen         (s_axi_awlen),
    .s_axi_awsize        (s_axi_awsize),
    .s_axi_awburst       (s_axi_awburst),
    .s_axi_wvalid        (s_axi_wvalid),
    .s_axi_wready        (s_axi_wready),
    .s_axi_wdata         (s_axi_wdata),
    .s_axi_wstrb         (s_axi_wstrb),
    .s_axi_wlast         (s_axi_wlast),
    .s_axi_bvalid        (s_axi_bvalid),
    .s_axi_bready        (s_axi_bready),
    .s_axi_bresp         (s_axi_bresp),
    .s_axi_bid           (s_axi_bid),
    .s_axi_arvalid       (s_axi_arvalid),
    .s_axi_arready       (s_axi_arready),
    .s_axi_arid          (s_axi_arid),
    .s_axi_araddr        (s_axi_araddr),
    .s_axi_arlen         (s_axi_arlen),
    .s_axi_arsize        (s_axi_arsize),
    .s_axi_arburst       (s_axi_arburst),
    .s_axi_rvalid        (s_axi_rvalid),
    .s_axi_rready        (s_axi_rready),
    .s_axi_rdata         (s_axi_rdata),
    .s_axi_rresp         (s_axi_rresp),
    .s_axi_rid           (s_axi_rid),
    .s_axi_rlast         (s_axi_rlast),

    .sr_req              (sr_req),
    .ref_req             (ref_req),
    .sr_ack              (sr_ack),
    .ref_ack             (ref_ack),
    .burst               (burst),

    .O_ddr_addr          (O_ddr_addr),
    .O_ddr_ba            (O_ddr_ba),
    .O_ddr_cs_n          (O_ddr_cs_n),
    .O_ddr_ras_n         (O_ddr_ras_n),
    .O_ddr_cas_n         (O_ddr_cas_n),
    .O_ddr_we_n          (O_ddr_we_n),
    .O_ddr_clk           (O_ddr_clk),
    .O_ddr_clk_n         (O_ddr_clk_n),
    .O_ddr_cke           (O_ddr_cke),
    .O_ddr_odt           (O_ddr_odt),
    .O_ddr_reset_n       (O_ddr_reset_n),
    .O_ddr_dqm           (O_ddr_dqm),
    .IO_ddr_dq           (IO_ddr_dq),
    .IO_ddr_dqs          (IO_ddr_dqs),
    .IO_ddr_dqs_n        (IO_ddr_dqs_n)
);

endmodule
