// ================================================================
// GW5AT-60K A2-1: DDR3 framebuffer -> ADV7513 HDMI 1080p60
//
// Function:
//   1) DDR3 AXI IP init at 400MHz memory_clk / 100MHz clk_out
//   2) AXI writer fills DDR with a 1920x1080 RGB888 colorbar framebuffer
//      Pixel storage format: 32'h00RRGGBB, 4 bytes per pixel
//   3) AXI reader reads DDR back through an async FIFO
//   4) HDMI pixel domain consumes FIFO data and drives ADV7513
//
// DDR3 IP assumptions:
//   DDR3_Memory_Interface_Top
//   AXI4 enabled
//   DQ Width = 32
//   Dram Width = 16
//   Row Address = 14
//   Column Address = 10
//   Memory Clock = 400MHz
//   CLK Ratio = 1:4
//   AXI Data Width = 256bit
//   AXI Address Width = 29bit
//
// Required external generated/source modules:
//   Gowin_PLL_DDR
//   Gowin_PLL                 // HDMI pixel PLL, 50MHz -> 148.5MHz
//   DDR3_Memory_Interface_Top
//   pll_mDRP_intf
//   adv7513_iic_init
//   I2C_MASTER_Top
// ================================================================

module top (
    input  wire        clk,       // 50MHz board clock
    input  wire        rst_n,     // active-low key reset

    output wire [1:0]  O_led,

    // ADV7513 HDMI TX
    inout  wire        IO_adv7513_scl,
    inout  wire        IO_adv7513_sda,
    output wire        O_adv7513_clk,
    output wire        O_adv7513_vs,
    output wire        O_adv7513_hs,
    output wire        O_adv7513_de,
    output wire [23:0] O_adv7513_data,

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
// 1080P60 timing
// ================================================================
localparam [15:0] H_TOTAL  = 16'd2200;
localparam [15:0] H_SYNC   = 16'd44;
localparam [15:0] H_BPORCH = 16'd148;
localparam [15:0] H_RES    = 16'd1920;
localparam [15:0] V_TOTAL  = 16'd1125;
localparam [15:0] V_SYNC   = 16'd5;
localparam [15:0] V_BPORCH = 16'd36;
localparam [15:0] V_RES    = 16'd1080;

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
// HDMI pixel PLL: 50MHz -> 148.5MHz
// ================================================================
wire pixel_clk;
wire pixel_pll_lock;

Gowin_PLL u_pixel_pll (
    .clkin  (clk),
    .clkout0(pixel_clk),
    .lock   (pixel_pll_lock),
    .mdclk  (clk),
    .reset  (!rst_n_50m)
);

reg [3:0] pixel_rst_sync;
always @(posedge pixel_clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        pixel_rst_sync <= 4'b0000;
    else if (!pixel_pll_lock)
        pixel_rst_sync <= 4'b0000;
    else
        pixel_rst_sync <= {pixel_rst_sync[2:0], 1'b1};
end
wire pixel_rst_n = pixel_rst_sync[3];

// ================================================================
// DDR PLL + pll_stop handling
// ================================================================
wire        pll_stop;
wire        ddr_pll_lock_raw;
wire        memory_clk;
wire [7:0]  mdrp_rdata;
wire [1:0]  mdrp_op;
wire        mdrp_inc;
wire [7:0]  mdrp_wdata;

reg [15:0] pll_lock_shift;
reg        pll_stop_d;
reg        pll_mdrp_wr;

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
wire        ddr_rst;
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
wire [31:0] s_axi_wstrb;
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
wire        burst   = 1'b0; // fixed BL8; OTF unused

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
// HDMI timing generator
// ================================================================
wire        hdmi_hs_raw;
wire        hdmi_vs_raw;
wire        hdmi_de_raw;
wire [11:0] hdmi_x;
wire [10:0] hdmi_y;
wire        hdmi_frame_start;

hdmi_1080p_timing u_hdmi_timing (
    .pixel_clk   (pixel_clk),
    .rst_n       (pixel_rst_n),
    .hs          (hdmi_hs_raw),
    .vs          (hdmi_vs_raw),
    .de          (hdmi_de_raw),
    .x           (hdmi_x),
    .y           (hdmi_y),
    .frame_start (hdmi_frame_start)
);

// ================================================================
// DDR framebuffer writer/reader
// ================================================================
wire [31:0] fb_pixel_word;
wire        fb_fill_done;
wire        fb_display_started;
wire        fb_error;
wire        fb_underflow_seen;
wire [3:0]  fb_state;
wire [15:0] fb_wr_burst_idx;
wire [15:0] fb_rd_burst_idx;
wire [11:0] fb_fifo_wr_count;
wire [11:0] fb_fifo_rd_count;

axi_ddr3_framebuffer_a2_1 #(
    .AXI_ADDR_WIDTH (29),
    .AXI_DATA_WIDTH (256),
    .AXI_ID_WIDTH   (4),
    .H_RES          (1920),
    .V_RES          (1080),
    .BURST_BEATS    (16),
    .FIFO_AW        (11)      // 2048 pixels async FIFO
) u_framebuffer (
    .axi_clk          (clk_out),
    .axi_rst          (ddr_rst | !init_calib_complete),
    .pixel_clk        (pixel_clk),
    .pixel_rst        (!pixel_rst_n),
    .display_de       (hdmi_de_raw),
    .display_frame_start(hdmi_frame_start),
    .pixel_word       (fb_pixel_word),

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

    .fill_done        (fb_fill_done),
    .display_started  (fb_display_started),
    .error_seen       (fb_error),
    .underflow_seen   (fb_underflow_seen),
    .state_dbg        (fb_state),
    .wr_burst_idx_dbg (fb_wr_burst_idx),
    .rd_burst_idx_dbg (fb_rd_burst_idx),
    .fifo_wr_count_dbg(fb_fifo_wr_count),
    .fifo_rd_count_dbg(fb_fifo_rd_count)
);

// ================================================================
// HDMI output data selection
// Before DDR framebuffer is ready, output full white.
// After DDR framebuffer starts, output DDR pixels.
// ================================================================
// One-cycle timing/data alignment for FIFO read latency.
reg        hdmi_hs_d;
reg        hdmi_vs_d;
reg        hdmi_de_d;
reg [23:0] hdmi_rgb_d;

wire [23:0] ddr_rgb = fb_pixel_word[23:0];
// Before DDR framebuffer is ready, force white screen.
// After framebuffer starts, show DDR pixels. Underflow is exposed by LED/ILA.
wire [23:0] fallback_rgb = 24'hFFFFFF;
wire        use_ddr_rgb = fb_display_started && !fb_error;

always @(posedge pixel_clk or negedge pixel_rst_n) begin
    if (!pixel_rst_n) begin
        hdmi_hs_d  <= 1'b0;
        hdmi_vs_d  <= 1'b0;
        hdmi_de_d  <= 1'b0;
        hdmi_rgb_d <= 24'h000000;
    end else begin
        hdmi_hs_d <= hdmi_hs_raw;
        hdmi_vs_d <= hdmi_vs_raw;
        hdmi_de_d <= hdmi_de_raw;
        if (hdmi_de_raw)
            hdmi_rgb_d <= use_ddr_rgb ? ddr_rgb : fallback_rgb;
        else
            hdmi_rgb_d <= 24'h000000;
    end
end

assign O_adv7513_clk  = pixel_clk;
assign O_adv7513_hs   = hdmi_hs_d;
assign O_adv7513_vs   = hdmi_vs_d;
assign O_adv7513_de   = hdmi_de_d;
assign O_adv7513_data = hdmi_rgb_d;

// ================================================================
// ADV7513 I2C init, copied from the verified GW60K_cb flow
// ================================================================
localparam [27:0] IIC_RESET_DELAY_CNT = 28'd50_000_000;
localparam [27:0] IIC_START_DELAY_CNT = 28'd100_000_000;

reg [27:0] iic_delay_cnt;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        iic_delay_cnt <= 28'd0;
    else if (iic_delay_cnt < IIC_START_DELAY_CNT)
        iic_delay_cnt <= iic_delay_cnt + 1'b1;
    else
        iic_delay_cnt <= iic_delay_cnt;
end

wire adv_iic_reset_n = rst_n_50m && (iic_delay_cnt >= IIC_RESET_DELAY_CNT);
wire adv_iic_start   = rst_n_50m && (iic_delay_cnt >= IIC_START_DELAY_CNT);

wire       TX_EN_7513;
wire [2:0] WADDR_7513;
wire [7:0] WDATA_7513;
wire       RX_EN_7513;
wire [2:0] RADDR_7513;
wire [7:0] RDATA_7513;

adv7513_iic_init u_adv7513_iic_init (
    .I_CLK       (clk),
    .I_RESETN    (adv_iic_reset_n),
    .start       (adv_iic_start),
    .O_TX_EN     (TX_EN_7513),
    .O_WADDR     (WADDR_7513),
    .O_WDATA     (WDATA_7513),
    .O_RX_EN     (RX_EN_7513),
    .O_RADDR     (RADDR_7513),
    .I_RDATA     (RDATA_7513),
    .cstate_flag (),
    .error_flag  ()
);

I2C_MASTER_Top u_i2c_master (
    .I_CLK     (clk),
    .I_RESETN  (adv_iic_reset_n),
    .I_TX_EN   (TX_EN_7513),
    .I_WADDR   (WADDR_7513),
    .I_WDATA   (WDATA_7513),
    .I_RX_EN   (RX_EN_7513),
    .I_RADDR   (RADDR_7513),
    .O_RDATA   (RDATA_7513),
    .O_IIC_INT (),
    .SCL       (IO_adv7513_scl),
    .SDA       (IO_adv7513_sda)
);

// ================================================================
// LEDs
//   O_led[0] = DDR init calibration done
//   O_led[1] = DDR framebuffer display OK; fast blink when error
// ================================================================
reg [25:0] led_cnt;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        led_cnt <= 26'd0;
    else
        led_cnt <= led_cnt + 1'b1;
end

assign O_led[0] = init_calib_complete;
assign O_led[1] = (fb_error || fb_underflow_seen) ? led_cnt[22] :
                  (fb_fill_done && fb_display_started) ? 1'b1 : led_cnt[25];

// ================================================================
// ILA search prefix: ila60a21_
// Use clk_out for DDR/AXI signals, pixel_clk for HDMI-domain signals.
// ================================================================
(* keep = "true" *) wire [31:0] ila60a21_top_version          = 32'h60A2_1001;
(* keep = "true" *) wire        ila60a21_ddr_pll_lock         = ddr_pll_lock;
(* keep = "true" *) wire        ila60a21_pixel_pll_lock       = pixel_pll_lock;
(* keep = "true" *) wire        ila60a21_ddr_rst              = ddr_rst;
(* keep = "true" *) wire        ila60a21_init_calib_complete  = init_calib_complete;
(* keep = "true" *) wire        ila60a21_fb_fill_done         = fb_fill_done;
(* keep = "true" *) wire        ila60a21_fb_display_started   = fb_display_started;
(* keep = "true" *) wire        ila60a21_fb_error             = fb_error;
(* keep = "true" *) wire        ila60a21_fb_underflow_seen    = fb_underflow_seen;
(* keep = "true" *) wire [3:0]  ila60a21_fb_state             = fb_state;
(* keep = "true" *) wire [15:0] ila60a21_wr_burst_idx         = fb_wr_burst_idx;
(* keep = "true" *) wire [15:0] ila60a21_rd_burst_idx         = fb_rd_burst_idx;
(* keep = "true" *) wire [11:0] ila60a21_fifo_wr_count        = fb_fifo_wr_count;
(* keep = "true" *) wire [11:0] ila60a21_fifo_rd_count        = fb_fifo_rd_count;
(* keep = "true" *) wire        ila60a21_awvalid              = s_axi_awvalid;
(* keep = "true" *) wire        ila60a21_awready              = s_axi_awready;
(* keep = "true" *) wire        ila60a21_wvalid               = s_axi_wvalid;
(* keep = "true" *) wire        ila60a21_wready               = s_axi_wready;
(* keep = "true" *) wire        ila60a21_wlast                = s_axi_wlast;
(* keep = "true" *) wire        ila60a21_bvalid               = s_axi_bvalid;
(* keep = "true" *) wire [1:0]  ila60a21_bresp                = s_axi_bresp;
(* keep = "true" *) wire        ila60a21_arvalid              = s_axi_arvalid;
(* keep = "true" *) wire        ila60a21_arready              = s_axi_arready;
(* keep = "true" *) wire        ila60a21_rvalid               = s_axi_rvalid;
(* keep = "true" *) wire        ila60a21_rready               = s_axi_rready;
(* keep = "true" *) wire        ila60a21_rlast                = s_axi_rlast;
(* keep = "true" *) wire [1:0]  ila60a21_rresp                = s_axi_rresp;
(* keep = "true" *) wire        ila60a21_hdmi_de              = hdmi_de_raw;
(* keep = "true" *) wire [11:0] ila60a21_hdmi_x               = hdmi_x;
(* keep = "true" *) wire [10:0] ila60a21_hdmi_y               = hdmi_y;
(* keep = "true" *) wire [23:0] ila60a21_hdmi_rgb             = hdmi_rgb_d;

endmodule

// ================================================================
// 1080p60 timing generator
// ================================================================
module hdmi_1080p_timing (
    input  wire        pixel_clk,
    input  wire        rst_n,
    output reg         hs,
    output reg         vs,
    output reg         de,
    output reg [11:0]  x,
    output reg [10:0]  y,
    output wire        frame_start
);
    localparam [15:0] H_TOTAL  = 16'd2200;
    localparam [15:0] H_SYNC   = 16'd44;
    localparam [15:0] H_BPORCH = 16'd148;
    localparam [15:0] H_RES    = 16'd1920;
    localparam [15:0] V_TOTAL  = 16'd1125;
    localparam [15:0] V_SYNC   = 16'd5;
    localparam [15:0] V_BPORCH = 16'd36;
    localparam [15:0] V_RES    = 16'd1080;
    localparam [15:0] H_ACT_ST = H_SYNC + H_BPORCH;
    localparam [15:0] V_ACT_ST = V_SYNC + V_BPORCH;

    reg [15:0] h_cnt;
    reg [15:0] v_cnt;

    assign frame_start = (h_cnt == 16'd0) && (v_cnt == 16'd0);

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 16'd0;
            v_cnt <= 16'd0;
        end else begin
            if (h_cnt == H_TOTAL - 1'b1) begin
                h_cnt <= 16'd0;
                if (v_cnt == V_TOTAL - 1'b1)
                    v_cnt <= 16'd0;
                else
                    v_cnt <= v_cnt + 1'b1;
            end else begin
                h_cnt <= h_cnt + 1'b1;
            end
        end
    end

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            hs <= 1'b0;
            vs <= 1'b0;
            de <= 1'b0;
            x  <= 12'd0;
            y  <= 11'd0;
        end else begin
            hs <= (h_cnt < H_SYNC);
            vs <= (v_cnt < V_SYNC);
            de <= (h_cnt >= H_ACT_ST) && (h_cnt < H_ACT_ST + H_RES) &&
                  (v_cnt >= V_ACT_ST) && (v_cnt < V_ACT_ST + V_RES);
            if ((h_cnt >= H_ACT_ST) && (h_cnt < H_ACT_ST + H_RES))
                x <= h_cnt - H_ACT_ST;
            else
                x <= 12'd0;
            if ((v_cnt >= V_ACT_ST) && (v_cnt < V_ACT_ST + V_RES))
                y <= v_cnt - V_ACT_ST;
            else
                y <= 11'd0;
        end
    end
endmodule

// ================================================================
// AXI DDR3 framebuffer writer/reader for A2-1
//
// IMPORTANT:
//   This version uses a generated Gowin asynchronous FIFO IP with
//   asymmetric width conversion:
//      write side: 256-bit, axi_clk
//      read side :  32-bit, pixel_clk
//
// Generate this FIFO IP as module name:
//      Video_FIFO_256to32
//
// Recommended FIFO IP settings:
//      Type              : Dual Clock FIFO / Async FIFO
//      Write width       : 256
//      Read width        : 32
//      Write depth       : 1024 or larger
//      Read depth        : 8192 or larger
//      FWFT / Show Ahead : ON
//      Almost Empty      : enabled, threshold about 1024 read words
//      Almost Full       : enabled, threshold about 896 write words
//
// Expected ports:
//      Data[255:0], WrClk, RdClk, WrEn, RdEn, Reset,
//      Q[31:0], Empty, Full, Almost_Empty, Almost_Full
//
// If your generated FIFO has different port names, only adjust the
// Video_FIFO_256to32 instance below.
// ================================================================
module axi_ddr3_framebuffer_a2_1 #(
    parameter AXI_ADDR_WIDTH = 29,
    parameter AXI_DATA_WIDTH = 256,
    parameter AXI_ID_WIDTH   = 4,
    parameter H_RES          = 1920,
    parameter V_RES          = 1080,
    parameter BURST_BEATS    = 16,
    parameter FIFO_AW        = 11
)(
    input  wire                         axi_clk,
    input  wire                         axi_rst,
    input  wire                         pixel_clk,
    input  wire                         pixel_rst,
    input  wire                         display_de,
    input  wire                         display_frame_start,
    output wire [31:0]                  pixel_word,

    output reg                          m_axi_awvalid,
    input  wire                         m_axi_awready,
    output wire [AXI_ID_WIDTH-1:0]      m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0]    m_axi_awaddr,
    output wire [7:0]                   m_axi_awlen,
    output wire [2:0]                   m_axi_awsize,
    output wire [1:0]                   m_axi_awburst,
    output reg                          m_axi_wvalid,
    input  wire                         m_axi_wready,
    output wire [AXI_DATA_WIDTH-1:0]    m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0]  m_axi_wstrb,
    output wire                         m_axi_wlast,
    input  wire                         m_axi_bvalid,
    output reg                          m_axi_bready,
    input  wire [1:0]                   m_axi_bresp,
    input  wire [AXI_ID_WIDTH-1:0]      m_axi_bid,

    output reg                          m_axi_arvalid,
    input  wire                         m_axi_arready,
    output wire [AXI_ID_WIDTH-1:0]      m_axi_arid,
    output wire [AXI_ADDR_WIDTH-1:0]    m_axi_araddr,
    output wire [7:0]                   m_axi_arlen,
    output wire [2:0]                   m_axi_arsize,
    output wire [1:0]                   m_axi_arburst,
    input  wire                         m_axi_rvalid,
    output reg                          m_axi_rready,
    input  wire [AXI_DATA_WIDTH-1:0]    m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire [AXI_ID_WIDTH-1:0]      m_axi_rid,
    input  wire                         m_axi_rlast,

    output reg                          fill_done,
    output reg                          display_started,
    output reg                          error_seen,
    output reg                          underflow_seen,
    output reg [3:0]                    state_dbg,
    output reg [15:0]                   wr_burst_idx_dbg,
    output reg [15:0]                   rd_burst_idx_dbg,
    output wire [FIFO_AW:0]             fifo_wr_count_dbg,
    output wire [FIFO_AW:0]             fifo_rd_count_dbg
);
    localparam integer PIXELS_PER_BEAT  = AXI_DATA_WIDTH / 32;          // 8 pixels per 256-bit AXI beat
    localparam integer PIXELS_PER_BURST = PIXELS_PER_BEAT * BURST_BEATS; // 128 pixels per burst
    localparam integer BURSTS_PER_LINE  = H_RES / PIXELS_PER_BURST;      // 15
    localparam integer TOTAL_BURSTS     = BURSTS_PER_LINE * V_RES;       // 16200
    localparam [7:0] AXI_LEN            = BURST_BEATS - 1;

    localparam [3:0]
        ST_WR_AW      = 4'd0,
        ST_WR_W       = 4'd1,
        ST_WR_B       = 4'd2,
        ST_RD_WAIT    = 4'd3,
        ST_RD_AR      = 4'd4,
        ST_RD_R       = 4'd5,
        ST_ERROR      = 4'd15;

    assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_awlen   = AXI_LEN;
    assign m_axi_arlen   = AXI_LEN;
    assign m_axi_awsize  = 3'b101; // 256bit = 32 bytes
    assign m_axi_arsize  = 3'b101;
    assign m_axi_awburst = 2'b01;  // INCR
    assign m_axi_arburst = 2'b01;
    assign m_axi_wstrb   = {AXI_DATA_WIDTH/8{1'b1}};

    reg [3:0]  state;
    reg [15:0] wr_burst_idx;
    reg [15:0] rd_burst_idx;
    reg [7:0]  wr_beat_idx;
    reg [11:0] wr_x;
    reg [10:0] wr_y;

    // One AXI burst = 16 beats * 32 bytes = 512 bytes.
    assign m_axi_awaddr = {{(AXI_ADDR_WIDTH-25){1'b0}}, wr_burst_idx, 9'b0};
    assign m_axi_araddr = {{(AXI_ADDR_WIDTH-25){1'b0}}, rd_burst_idx, 9'b0};
    assign m_axi_wlast  = m_axi_wvalid && (wr_beat_idx == AXI_LEN);
    assign m_axi_wdata  = pack_8pixels(wr_x, wr_y);

    function [23:0] color_at;
        input [11:0] x;
        input [10:0] y;
        begin
            if      (x < 12'd240)  color_at = 24'hFFFFFF;
            else if (x < 12'd480)  color_at = 24'hFFFF00;
            else if (x < 12'd720)  color_at = 24'h00FFFF;
            else if (x < 12'd960)  color_at = 24'h00FF00;
            else if (x < 12'd1200) color_at = 24'hFF00FF;
            else if (x < 12'd1440) color_at = 24'hFF0000;
            else if (x < 12'd1680) color_at = 24'h0000FF;
            else                   color_at = 24'h000000;
        end
    endfunction

    function [255:0] pack_8pixels;
        input [11:0] x0;
        input [10:0] y0;
        begin
            pack_8pixels = {
                8'h00, color_at(x0 + 12'd7, y0),
                8'h00, color_at(x0 + 12'd6, y0),
                8'h00, color_at(x0 + 12'd5, y0),
                8'h00, color_at(x0 + 12'd4, y0),
                8'h00, color_at(x0 + 12'd3, y0),
                8'h00, color_at(x0 + 12'd2, y0),
                8'h00, color_at(x0 + 12'd1, y0),
                8'h00, color_at(x0 + 12'd0, y0)
            };
        end
    endfunction

    // ================================================================
    // Generated async FIFO IP: 256-bit write side -> 32-bit read side
    // ================================================================
    wire        fifo_rst = axi_rst | !fill_done;
    reg         fifo_wr_en;
    wire [255:0] fifo_wr_data = m_axi_rdata;
    wire        fifo_rd_en;
    wire [31:0] fifo_rd_data;
    wire        fifo_empty;
    wire        fifo_full;
    wire        fifo_aempty;
    wire        fifo_afull;

    // The previous hand-coded FIFO wrote one 32-bit pixel per 100MHz axi_clk,
    // which cannot feed a 148.5MHz active video stream. This FIFO writes one
    // whole 256-bit DDR beat per axi_clk and performs width conversion internally.
    Video_FIFO_256to32 u_video_fifo_256to32 (
        .Data         (fifo_wr_data),
        .WrClk        (axi_clk),
        .RdClk        (pixel_clk),
        .WrEn         (fifo_wr_en),
        .RdEn         (fifo_rd_en),
        .Reset        (fifo_rst),
        .Q            (fifo_rd_data),
        .Empty        (fifo_empty),
        .Full         (fifo_full),
        .Almost_Empty (fifo_aempty),
        .Almost_Full  (fifo_afull)
    );

    assign pixel_word = fifo_rd_data;
    assign fifo_rd_en = display_started && display_de && !fifo_empty;

    // Count debug is not available from the minimal FIFO port set.
    assign fifo_wr_count_dbg = {FIFO_AW+1{1'b0}};
    assign fifo_rd_count_dbg = {FIFO_AW+1{1'b0}};

    // -----------------------------
    // Pixel-domain display enable
    // -----------------------------
    reg fill_done_p1;
    reg fill_done_p2;
    always @(posedge pixel_clk or posedge pixel_rst) begin
        if (pixel_rst) begin
            fill_done_p1      <= 1'b0;
            fill_done_p2      <= 1'b0;
            display_started   <= 1'b0;
            underflow_seen    <= 1'b0;
        end else begin
            fill_done_p1 <= fill_done;
            fill_done_p2 <= fill_done_p1;

            // Start displaying DDR framebuffer only on a clean frame boundary,
            // and only after the read-side FIFO has passed the Almost_Empty threshold.
            if (!display_started && fill_done_p2 && !fifo_aempty && display_frame_start)
                display_started <= 1'b1;

            if (display_started && display_de && fifo_empty)
                underflow_seen <= 1'b1;
        end
    end

    // -----------------------------
    // AXI writer / reader FSM
    // -----------------------------
    always @(posedge axi_clk) begin
        if (axi_rst) begin
            state             <= ST_WR_AW;
            m_axi_awvalid     <= 1'b0;
            m_axi_wvalid      <= 1'b0;
            m_axi_bready      <= 1'b0;
            m_axi_arvalid     <= 1'b0;
            m_axi_rready      <= 1'b0;
            fill_done         <= 1'b0;
            error_seen        <= 1'b0;
            wr_burst_idx      <= 16'd0;
            rd_burst_idx      <= 16'd0;
            wr_beat_idx       <= 8'd0;
            wr_x              <= 12'd0;
            wr_y              <= 11'd0;
            fifo_wr_en        <= 1'b0;
            state_dbg         <= ST_WR_AW;
            wr_burst_idx_dbg  <= 16'd0;
            rd_burst_idx_dbg  <= 16'd0;
        end else begin
            fifo_wr_en <= 1'b0;
            state_dbg <= state;
            wr_burst_idx_dbg <= wr_burst_idx;
            rd_burst_idx_dbg <= rd_burst_idx;

            case (state)
                ST_WR_AW: begin
                    m_axi_awvalid <= 1'b1;
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b1;
                        wr_beat_idx   <= 8'd0;
                        state         <= ST_WR_W;
                    end
                end

                ST_WR_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        if (wr_x == H_RES - 8) begin
                            wr_x <= 12'd0;
                            if (wr_y == V_RES - 1)
                                wr_y <= 11'd0;
                            else
                                wr_y <= wr_y + 1'b1;
                        end else begin
                            wr_x <= wr_x + 12'd8;
                        end

                        if (wr_beat_idx == AXI_LEN) begin
                            m_axi_wvalid <= 1'b0;
                            m_axi_bready <= 1'b1;
                            state        <= ST_WR_B;
                        end else begin
                            wr_beat_idx <= wr_beat_idx + 1'b1;
                        end
                    end
                end

                ST_WR_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        if (m_axi_bresp != 2'b00) begin
                            error_seen <= 1'b1;
                            state      <= ST_ERROR;
                        end else if (wr_burst_idx == TOTAL_BURSTS - 1) begin
                            fill_done    <= 1'b1;
                            rd_burst_idx <= 16'd0;
                            state        <= ST_RD_WAIT;
                        end else begin
                            wr_burst_idx <= wr_burst_idx + 1'b1;
                            state        <= ST_WR_AW;
                        end
                    end
                end

                ST_RD_WAIT: begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    if (fill_done && !fifo_afull) begin
                        m_axi_arvalid <= 1'b1;
                        state         <= ST_RD_AR;
                    end
                end

                ST_RD_AR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= !fifo_full;
                        state         <= ST_RD_R;
                    end
                end

                ST_RD_R: begin
                    // Backpressure DDR read data if the FIFO is full.
                    m_axi_rready <= !fifo_full;

                    if (m_axi_rvalid && m_axi_rready) begin
                        if (m_axi_rresp != 2'b00) begin
                            error_seen   <= 1'b1;
                            m_axi_rready <= 1'b0;
                            state        <= ST_ERROR;
                        end else begin
                            fifo_wr_en <= 1'b1;

                            if (m_axi_rlast) begin
                                m_axi_rready <= 1'b0;
                                if (rd_burst_idx == TOTAL_BURSTS - 1)
                                    rd_burst_idx <= 16'd0;
                                else
                                    rd_burst_idx <= rd_burst_idx + 1'b1;
                                state <= ST_RD_WAIT;
                            end
                        end
                    end
                end

                ST_ERROR: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    error_seen    <= 1'b1;
                end

                default: begin
                    state <= ST_ERROR;
                end
            endcase
        end
    end
endmodule
