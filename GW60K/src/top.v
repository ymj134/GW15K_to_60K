// ================================================================
// GW5AT-60K DDR3 AXI4 standalone read/write test top
// A2-0: DDR3 AXI independent verification
//
// Target DDR3 IP configuration assumed:
//   DDR3_Memory_Interface_Top
//   AXI4 Interface enabled
//   DQ Width       = 32
//   Dram Width     = 16
//   Row Address    = 14
//   Column Address = 10
//   Memory Clock   = 400MHz
//   CLK Ratio      = 1:4
//   AXI Data Width = 256bit
//   AXI Addr Width = 29bit
//
// External modules required:
//   1) DDR3_Memory_Interface_Top        -- generated DDR3 IP
//   2) Gowin_PLL_DDR                    -- generated DDR PLL, 50MHz -> clkout2=400MHz
//   3) pll_mDRP_intf                    -- from Gowin DDR3 reference design if PLL needs mDRP pll_stop handling
//
// LED:
//   O_led[0] = init_calib_complete
//   O_led[1] = pass: ON, fail: fast blink, busy: slow blink
// ================================================================

module top (
    input  wire        clk,       // 50MHz board clock
    input  wire        rst_n,     // active-low key reset
    output wire [1:0]  O_led,

    // DDR3 physical interface
    output wire [13:0] O_ddr_addr,
    output wire [2:0]  O_ddr_ba,
    output wire        O_ddr_cs_n,
    output wire        O_ddr_ras_n,
    output wire        O_ddr_cas_n,
    output wire        O_ddr_we_n,
    output wire        O_ddr_clk,
    output wire        O_ddr_clk_n,
    output wire        O_ddr_cke,
    output wire        O_ddr_odt,
    output wire        O_ddr_reset_n,
    output wire [3:0]  O_ddr_dqm,
    inout  wire [31:0] IO_ddr_dq,
    inout  wire [3:0]  IO_ddr_dqs,
    inout  wire [3:0]  IO_ddr_dqs_n
);

// ================================================================
// 50MHz reset sync
// ================================================================
reg [3:0] rst_sync;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rst_sync <= 4'b0000;
    else
        rst_sync <= {rst_sync[2:0], 1'b1};
end
wire rst_n_50m = rst_sync[3];

// ================================================================
// DDR PLL + pll_stop handling
//
// Important:
// For Gowin DDR3 on GW5A/GW5AT devices, memory_clk should come from
// PLL clkout2 in the official flow. The pll_stop output of DDR3 IP is
// used to control/switch memory_clk. The official 60K reference design
// uses pll_mDRP_intf for this purpose.
// ================================================================
wire        pll_stop;
wire        ddr_pll_lock_raw;
wire        memory_clk;
wire [7:0]  mdrp_rdata;
wire [1:0]  mdrp_op;
wire        mdrp_inc;
wire [7:0]  mdrp_wdata;
reg  [15:0] pll_lock_shift;
reg         pll_stop_d;
reg         pll_mdrp_wr;

always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        pll_lock_shift <= 16'd0;
    else
        pll_lock_shift <= {pll_lock_shift[14:0], ddr_pll_lock_raw};
end
wire ddr_pll_lock = pll_lock_shift[15];

always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m) begin
        pll_stop_d  <= 1'b0;
        pll_mdrp_wr <= 1'b0;
    end else begin
        pll_stop_d  <= pll_stop;
        pll_mdrp_wr <= ddr_pll_lock && (pll_stop ^ pll_stop_d);
    end
end

// Generate this PLL IP with module name Gowin_PLL_DDR.
// Required output: clkout2 = 400MHz memory_clk.
// Keep the mDRP ports if your generated PLL follows the official DDR3 reference design.
Gowin_PLL_DDR u_ddr_pll (
    .lock            (ddr_pll_lock_raw),
    .clkout0         (),
    .clkout2         (memory_clk),
    .mdrdo           (mdrp_rdata),
    .clkin           (clk),
    .pll_init_bypass (ddr_pll_lock),
    .reset           (!rst_n_50m),
    .mdclk           (clk),
    .mdopc           (mdrp_op),
    .mdainc          (mdrp_inc),
    .mdwdi           (mdrp_wdata)
);

pll_mDRP_intf u_pll_mDRP_intf (
    .clk        (clk),
    .rst_n      (rst_n_50m),
    .pll_lock   (ddr_pll_lock),
    .wr         (pll_mdrp_wr),
    .mdrp_inc   (mdrp_inc),
    .mdrp_op    (mdrp_op),
    .mdrp_wdata (mdrp_wdata),
    .mdrp_rdata (mdrp_rdata)
);

// ================================================================
// DDR3 IP AXI wires
// ================================================================
wire        clk_out;
wire        ddr_rst;                // high-active user reset from DDR IP
wire        init_calib_complete;

wire        s_axi_awvalid;
wire        s_axi_awready;
wire [3:0]  s_axi_awid;
wire [28:0] s_axi_awaddr;
wire [7:0]  s_axi_awlen;
wire [2:0]  s_axi_awsize;
wire [1:0]  s_axi_awburst;

wire        s_axi_wvalid;
wire        s_axi_wready;
wire [255:0] s_axi_wdata;
wire [31:0]  s_axi_wstrb;
wire        s_axi_wlast;

wire        s_axi_bvalid;
wire        s_axi_bready;
wire [1:0]  s_axi_bresp;
wire [3:0]  s_axi_bid;

wire        s_axi_arvalid;
wire        s_axi_arready;
wire [3:0]  s_axi_arid;
wire [28:0] s_axi_araddr;
wire [7:0]  s_axi_arlen;
wire [2:0]  s_axi_arsize;
wire [1:0]  s_axi_arburst;

wire        s_axi_rvalid;
wire        s_axi_rready;
wire [255:0] s_axi_rdata;
wire [1:0]  s_axi_rresp;
wire [3:0]  s_axi_rid;
wire        s_axi_rlast;

wire        sr_req  = 1'b0;
wire        ref_req = 1'b0;
wire        sr_ack;
wire        ref_ack;
wire        burst   = 1'b0;          // fixed BL8, OTF burst input unused

DDR3_Memory_Interface_Top u_ddr3 (
    .clk                 (clk),
    .pll_stop            (pll_stop),
    .memory_clk          (memory_clk),
    .pll_lock            (ddr_pll_lock),
    .rst_n               (rst_n_50m),
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

// ================================================================
// AXI DDR read/write tester
// ================================================================
wire        test_done;
wire        test_pass;
wire        test_fail;
wire [3:0]  test_state;
wire [15:0] test_wr_burst_idx;
wire [15:0] test_rd_burst_idx;
wire [7:0]  test_beat_idx;
wire [7:0]  test_err_code;
wire [28:0] test_err_addr;
wire [255:0] test_err_expected;
wire [255:0] test_err_actual;

axi_ddr3_basic_tester #(
    .ADDR_WIDTH       (29),
    .DATA_WIDTH       (256),
    .ID_WIDTH         (4),
    .BURST_BEATS      (16),
    .TOTAL_BURSTS     (1024)       // 1024 * 16 * 32B = 512KB test range
) u_axi_ddr3_basic_tester (
    .clk              (clk_out),
    .rst              (ddr_rst | !init_calib_complete),
    .start            (init_calib_complete),

    .m_axi_awvalid    (s_axi_awvalid),
    .m_axi_awready    (s_axi_awready),
    .m_axi_awid       (s_axi_awid),
    .m_axi_awaddr     (s_axi_awaddr),
    .m_axi_awlen      (s_axi_awlen),
    .m_axi_awsize     (s_axi_awsize),
    .m_axi_awburst    (s_axi_awburst),

    .m_axi_wvalid     (s_axi_wvalid),
    .m_axi_wready     (s_axi_wready),
    .m_axi_wdata      (s_axi_wdata),
    .m_axi_wstrb      (s_axi_wstrb),
    .m_axi_wlast      (s_axi_wlast),

    .m_axi_bvalid     (s_axi_bvalid),
    .m_axi_bready     (s_axi_bready),
    .m_axi_bresp      (s_axi_bresp),
    .m_axi_bid        (s_axi_bid),

    .m_axi_arvalid    (s_axi_arvalid),
    .m_axi_arready    (s_axi_arready),
    .m_axi_arid       (s_axi_arid),
    .m_axi_araddr     (s_axi_araddr),
    .m_axi_arlen      (s_axi_arlen),
    .m_axi_arsize     (s_axi_arsize),
    .m_axi_arburst    (s_axi_arburst),

    .m_axi_rvalid     (s_axi_rvalid),
    .m_axi_rready     (s_axi_rready),
    .m_axi_rdata      (s_axi_rdata),
    .m_axi_rresp      (s_axi_rresp),
    .m_axi_rid        (s_axi_rid),
    .m_axi_rlast      (s_axi_rlast),

    .test_done        (test_done),
    .test_pass        (test_pass),
    .test_fail        (test_fail),
    .test_state       (test_state),
    .wr_burst_idx     (test_wr_burst_idx),
    .rd_burst_idx     (test_rd_burst_idx),
    .beat_idx         (test_beat_idx),
    .err_code         (test_err_code),
    .err_addr         (test_err_addr),
    .err_expected     (test_err_expected),
    .err_actual       (test_err_actual)
);

// ================================================================
// LEDs
// ================================================================
reg [25:0] led_cnt_50m;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        led_cnt_50m <= 26'd0;
    else
        led_cnt_50m <= led_cnt_50m + 1'b1;
end

assign O_led[0] = init_calib_complete;
assign O_led[1] = test_pass ? 1'b1 :
                  test_fail ? led_cnt_50m[22] :
                              led_cnt_50m[25];

// ================================================================
// ILA search prefix: ila60ddr_
// Add these wires to GAO/ILA. Use clk_out as sampling clock.
// ================================================================
(* keep = "true" *) wire [31:0] ila60ddr_top_version          = 32'h60D3_A200;
(* keep = "true" *) wire        ila60ddr_rst_n_50m            = rst_n_50m;
(* keep = "true" *) wire        ila60ddr_pll_lock_raw         = ddr_pll_lock_raw;
(* keep = "true" *) wire        ila60ddr_pll_lock             = ddr_pll_lock;
(* keep = "true" *) wire        ila60ddr_pll_stop             = pll_stop;
(* keep = "true" *) wire        ila60ddr_ddr_rst              = ddr_rst;
(* keep = "true" *) wire        ila60ddr_init_calib_complete  = init_calib_complete;
(* keep = "true" *) wire        ila60ddr_test_done            = test_done;
(* keep = "true" *) wire        ila60ddr_test_pass            = test_pass;
(* keep = "true" *) wire        ila60ddr_test_fail            = test_fail;
(* keep = "true" *) wire [3:0]  ila60ddr_test_state           = test_state;
(* keep = "true" *) wire [15:0] ila60ddr_wr_burst_idx         = test_wr_burst_idx;
(* keep = "true" *) wire [15:0] ila60ddr_rd_burst_idx         = test_rd_burst_idx;
(* keep = "true" *) wire [7:0]  ila60ddr_beat_idx             = test_beat_idx;
(* keep = "true" *) wire [7:0]  ila60ddr_err_code             = test_err_code;
(* keep = "true" *) wire [28:0] ila60ddr_err_addr             = test_err_addr;
(* keep = "true" *) wire [255:0] ila60ddr_err_expected        = test_err_expected;
(* keep = "true" *) wire [255:0] ila60ddr_err_actual          = test_err_actual;

(* keep = "true" *) wire        ila60ddr_awvalid              = s_axi_awvalid;
(* keep = "true" *) wire        ila60ddr_awready              = s_axi_awready;
(* keep = "true" *) wire [28:0] ila60ddr_awaddr               = s_axi_awaddr;
(* keep = "true" *) wire [7:0]  ila60ddr_awlen                = s_axi_awlen;
(* keep = "true" *) wire        ila60ddr_wvalid               = s_axi_wvalid;
(* keep = "true" *) wire        ila60ddr_wready               = s_axi_wready;
(* keep = "true" *) wire        ila60ddr_wlast                = s_axi_wlast;
(* keep = "true" *) wire        ila60ddr_bvalid               = s_axi_bvalid;
(* keep = "true" *) wire        ila60ddr_bready               = s_axi_bready;
(* keep = "true" *) wire [1:0]  ila60ddr_bresp                = s_axi_bresp;
(* keep = "true" *) wire        ila60ddr_arvalid              = s_axi_arvalid;
(* keep = "true" *) wire        ila60ddr_arready              = s_axi_arready;
(* keep = "true" *) wire [28:0] ila60ddr_araddr               = s_axi_araddr;
(* keep = "true" *) wire [7:0]  ila60ddr_arlen                = s_axi_arlen;
(* keep = "true" *) wire        ila60ddr_rvalid               = s_axi_rvalid;
(* keep = "true" *) wire        ila60ddr_rready               = s_axi_rready;
(* keep = "true" *) wire        ila60ddr_rlast                = s_axi_rlast;
(* keep = "true" *) wire [1:0]  ila60ddr_rresp                = s_axi_rresp;

endmodule


// ================================================================
// Simple AXI4 DDR tester
// Writes deterministic 256-bit pattern bursts, then reads them back.
// ================================================================
module axi_ddr3_basic_tester #(
    parameter ADDR_WIDTH   = 29,
    parameter DATA_WIDTH   = 256,
    parameter ID_WIDTH     = 4,
    parameter BURST_BEATS  = 16,
    parameter TOTAL_BURSTS = 1024
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  start,

    output reg                   m_axi_awvalid,
    input  wire                  m_axi_awready,
    output wire [ID_WIDTH-1:0]   m_axi_awid,
    output wire [ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]            m_axi_awlen,
    output wire [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,

    output reg                   m_axi_wvalid,
    input  wire                  m_axi_wready,
    output wire [DATA_WIDTH-1:0] m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                  m_axi_wlast,

    input  wire                  m_axi_bvalid,
    output reg                   m_axi_bready,
    input  wire [1:0]            m_axi_bresp,
    input  wire [ID_WIDTH-1:0]   m_axi_bid,

    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,
    output wire [ID_WIDTH-1:0]   m_axi_arid,
    output wire [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,

    input  wire                  m_axi_rvalid,
    output reg                   m_axi_rready,
    input  wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire [ID_WIDTH-1:0]   m_axi_rid,
    input  wire                  m_axi_rlast,

    output reg                   test_done,
    output reg                   test_pass,
    output reg                   test_fail,
    output reg [3:0]             test_state,
    output reg [15:0]            wr_burst_idx,
    output reg [15:0]            rd_burst_idx,
    output reg [7:0]             beat_idx,
    output reg [7:0]             err_code,
    output reg [ADDR_WIDTH-1:0]  err_addr,
    output reg [DATA_WIDTH-1:0]  err_expected,
    output reg [DATA_WIDTH-1:0]  err_actual
);

localparam [3:0]
    ST_IDLE  = 4'd0,
    ST_WR_AW = 4'd1,
    ST_WR_W  = 4'd2,
    ST_WR_B  = 4'd3,
    ST_RD_AR = 4'd4,
    ST_RD_R  = 4'd5,
    ST_PASS  = 4'd6,
    ST_FAIL  = 4'd7;

localparam [7:0] AXI_BURST_LEN = BURST_BEATS - 1;
localparam [2:0] AXI_SIZE_256B = 3'b101; // 2^5 = 32 bytes = 256 bits
localparam [1:0] AXI_BURST_INCR = 2'b01;

assign m_axi_awid    = {ID_WIDTH{1'b0}};
assign m_axi_arid    = {ID_WIDTH{1'b0}};
assign m_axi_awlen   = AXI_BURST_LEN;
assign m_axi_arlen   = AXI_BURST_LEN;
assign m_axi_awsize  = AXI_SIZE_256B;
assign m_axi_arsize  = AXI_SIZE_256B;
assign m_axi_awburst = AXI_BURST_INCR;
assign m_axi_arburst = AXI_BURST_INCR;
assign m_axi_wstrb   = {DATA_WIDTH/8{1'b1}};
assign m_axi_wlast   = m_axi_wvalid && (beat_idx == AXI_BURST_LEN);

assign m_axi_awaddr  = burst_addr(wr_burst_idx);
assign m_axi_araddr  = burst_addr(rd_burst_idx);
assign m_axi_wdata   = gen_data(wr_burst_idx, beat_idx);

function [ADDR_WIDTH-1:0] burst_addr;
    input [15:0] burst;
    begin
        // Each burst is 16 beats * 32 bytes = 512 bytes, so address step is 512.
        burst_addr = {{(ADDR_WIDTH-25){1'b0}}, burst, 9'b0};
    end
endfunction

function [255:0] gen_data;
    input [15:0] burst;
    input [7:0]  beat;
    reg   [31:0] seed;
    begin
        seed = {8'hA5, burst[7:0], beat[7:0], burst[15:8]};
        gen_data = {
            seed ^ 32'hF0F0_0007,
            seed ^ 32'hE1E1_0006,
            seed ^ 32'hD2D2_0005,
            seed ^ 32'hC3C3_0004,
            seed ^ 32'hB4B4_0003,
            seed ^ 32'hA5A5_0002,
            seed ^ 32'h9696_0001,
            seed ^ 32'h8787_0000
        };
    end
endfunction

wire [255:0] rd_expected = gen_data(rd_burst_idx, beat_idx);
wire [ADDR_WIDTH-1:0] rd_addr_now = burst_addr(rd_burst_idx) + {{(ADDR_WIDTH-8){1'b0}}, beat_idx, 5'b0};

always @(posedge clk) begin
    if (rst) begin
        test_state    <= ST_IDLE;
        m_axi_awvalid <= 1'b0;
        m_axi_wvalid  <= 1'b0;
        m_axi_bready  <= 1'b0;
        m_axi_arvalid <= 1'b0;
        m_axi_rready  <= 1'b0;
        test_done     <= 1'b0;
        test_pass     <= 1'b0;
        test_fail     <= 1'b0;
        wr_burst_idx  <= 16'd0;
        rd_burst_idx  <= 16'd0;
        beat_idx      <= 8'd0;
        err_code      <= 8'd0;
        err_addr      <= {ADDR_WIDTH{1'b0}};
        err_expected  <= {DATA_WIDTH{1'b0}};
        err_actual    <= {DATA_WIDTH{1'b0}};
    end else begin
        case (test_state)
            ST_IDLE: begin
                test_done     <= 1'b0;
                test_pass     <= 1'b0;
                test_fail     <= 1'b0;
                err_code      <= 8'd0;
                wr_burst_idx  <= 16'd0;
                rd_burst_idx  <= 16'd0;
                beat_idx      <= 8'd0;
                m_axi_awvalid <= 1'b0;
                m_axi_wvalid  <= 1'b0;
                m_axi_bready  <= 1'b0;
                m_axi_arvalid <= 1'b0;
                m_axi_rready  <= 1'b0;
                if (start) begin
                    test_state    <= ST_WR_AW;
                    m_axi_awvalid <= 1'b1;
                end
            end

            ST_WR_AW: begin
                if (m_axi_awvalid && m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b1;
                    beat_idx      <= 8'd0;
                    test_state    <= ST_WR_W;
                end
            end

            ST_WR_W: begin
                if (m_axi_wvalid && m_axi_wready) begin
                    if (beat_idx == AXI_BURST_LEN) begin
                        m_axi_wvalid <= 1'b0;
                        m_axi_bready <= 1'b1;
                        test_state   <= ST_WR_B;
                    end else begin
                        beat_idx <= beat_idx + 1'b1;
                    end
                end
            end

            ST_WR_B: begin
                if (m_axi_bvalid && m_axi_bready) begin
                    m_axi_bready <= 1'b0;
                    if (m_axi_bresp != 2'b00) begin
                        test_fail    <= 1'b1;
                        test_done    <= 1'b1;
                        err_code     <= 8'd1; // write response error
                        err_addr     <= burst_addr(wr_burst_idx);
                        err_expected <= {DATA_WIDTH{1'b0}};
                        err_actual   <= {{(DATA_WIDTH-2){1'b0}}, m_axi_bresp};
                        test_state   <= ST_FAIL;
                    end else if (wr_burst_idx == TOTAL_BURSTS-1) begin
                        rd_burst_idx  <= 16'd0;
                        beat_idx      <= 8'd0;
                        m_axi_arvalid <= 1'b1;
                        test_state    <= ST_RD_AR;
                    end else begin
                        wr_burst_idx  <= wr_burst_idx + 1'b1;
                        m_axi_awvalid <= 1'b1;
                        test_state    <= ST_WR_AW;
                    end
                end
            end

            ST_RD_AR: begin
                if (m_axi_arvalid && m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    beat_idx      <= 8'd0;
                    test_state    <= ST_RD_R;
                end
            end

            ST_RD_R: begin
                if (m_axi_rvalid && m_axi_rready) begin
                    if (m_axi_rresp != 2'b00) begin
                        test_fail    <= 1'b1;
                        test_done    <= 1'b1;
                        err_code     <= 8'd2; // read response error
                        err_addr     <= rd_addr_now;
                        err_expected <= rd_expected;
                        err_actual   <= m_axi_rdata;
                        m_axi_rready <= 1'b0;
                        test_state   <= ST_FAIL;
                    end else if (m_axi_rdata != rd_expected) begin
                        test_fail    <= 1'b1;
                        test_done    <= 1'b1;
                        err_code     <= 8'd3; // data mismatch
                        err_addr     <= rd_addr_now;
                        err_expected <= rd_expected;
                        err_actual   <= m_axi_rdata;
                        m_axi_rready <= 1'b0;
                        test_state   <= ST_FAIL;
                    end else if (m_axi_rlast != (beat_idx == AXI_BURST_LEN)) begin
                        test_fail    <= 1'b1;
                        test_done    <= 1'b1;
                        err_code     <= 8'd4; // rlast position error
                        err_addr     <= rd_addr_now;
                        err_expected <= rd_expected;
                        err_actual   <= m_axi_rdata;
                        m_axi_rready <= 1'b0;
                        test_state   <= ST_FAIL;
                    end else if (beat_idx == AXI_BURST_LEN) begin
                        m_axi_rready <= 1'b0;
                        if (rd_burst_idx == TOTAL_BURSTS-1) begin
                            test_pass  <= 1'b1;
                            test_done  <= 1'b1;
                            test_state <= ST_PASS;
                        end else begin
                            rd_burst_idx  <= rd_burst_idx + 1'b1;
                            beat_idx      <= 8'd0;
                            m_axi_arvalid <= 1'b1;
                            test_state    <= ST_RD_AR;
                        end
                    end else begin
                        beat_idx <= beat_idx + 1'b1;
                    end
                end
            end

            ST_PASS: begin
                test_pass <= 1'b1;
                test_done <= 1'b1;
            end

            ST_FAIL: begin
                test_fail <= 1'b1;
                test_done <= 1'b1;
            end

            default: begin
                test_state <= ST_IDLE;
            end
        endcase
    end
end

endmodule
