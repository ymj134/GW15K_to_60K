// ============================================================================
// GW5AT-60K B2: 15K RoraLink segmented video RX -> DDR3 framebuffer -> HDMI
// ----------------------------------------------------------------------------
// Function:
//   1) Receive segmented RGB888_32 packets from 15K through RoraLink 8B10B.
//   2) Depacketize payload words and aggregate 8 pixels into one 256-bit beat.
//   3) Cross rx_clk -> DDR clk_out through Video_WR_FIFO_256.
//   4) AXI-write received video into DDR framebuffer.
//   5) AXI-read DDR framebuffer through Video_FIFO_256to32.
//   6) Output 720p60 RGB888 through verified ADV7513 HDMI path.
//
// Required generated/source modules:
//   SerDes_Top                  // 60K RoraLink RX-only IP
//   DDR3_Memory_Interface_Top   // AXI4 DDR3 IP, 256-bit data
//   Gowin_PLL_DDR               // DDR memory_clk PLL
//   Gowin_PLL                   // HDMI 74.25MHz pixel PLL
//   pll_mDRP_intf
//   adv7513_iic_init
//   I2C_MASTER_Top
//   Video_FIFO_256to32          // DDR read FIFO: 256bit clk_out -> 32bit pixel_clk
//   Video_WR_FIFO_256           // NEW: RoraLink RX write FIFO: 256bit rx_clk -> 256bit clk_out
//
// Recommended Video_WR_FIFO_256 settings:
//   Dual-clock FIFO, FWFT, BSRAM
//   Write width = 256, Read width = 256
//   Write depth = 2048, Read depth = 2048
//   Data Number enabled: Wnum[11:0], Rnum[11:0]
//   Ports expected below: Data, Reset, WrClk, RdClk, WrEn, RdEn, Wnum, Rnum,
//                         Almost_Empty, Almost_Full, Q, Empty, Full
// ============================================================================

`timescale 1ns / 1ps

module top (
    input  wire        clk,       // 50MHz board clock
    input  wire        rst_n,
    output wire [1:0]  O_led,     // 60K board LEDs are active-low

    output wire        sfp_tx_disable1,
    output wire        sfp_tx_disable2,

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

assign sfp_tx_disable1 = 1'b0;
assign sfp_tx_disable2 = 1'b0;

// --------------------------------------------------------------------------
// 50MHz reset sync
// --------------------------------------------------------------------------
reg [3:0] rst_sync;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rst_sync <= 4'b0000;
    else
        rst_sync <= {rst_sync[2:0], 1'b1};
end
wire rst_n_50m = rst_sync[3];
wire global_rst = ~rst_n_50m;

// --------------------------------------------------------------------------
// HDMI pixel PLL: 50MHz -> 74.25MHz
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// DDR PLL + pll_stop handling
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// DDR3 IP AXI wires
// --------------------------------------------------------------------------
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
wire        burst   = 1'b0;

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

// --------------------------------------------------------------------------
// RoraLink RX-only IP wires
// --------------------------------------------------------------------------
wire        rl_link_reset;
wire        rl_sys_reset;
wire [31:0] rl_rx_data;
wire [3:0]  rl_rx_strb;
wire        rl_rx_valid;
wire        rl_rx_last;
wire        rl_crc_pass_fail_n;
wire        rl_crc_valid;
wire        rl_hard_err;
wire        rl_soft_err;
wire        rl_frame_err;
wire        rl_channel_up;
wire        rl_lane_up;
wire        rl_rx_clk;
wire        rl_gt_pll_ok;
wire        rl_gt_rx_align_link;
wire        rl_gt_rx_pma_lock;
wire        rl_gt_rx_k_lock;

SerDes_Top u_serdes_top(
    .RoraLink_8B10B_Top_link_reset_o         (rl_link_reset),
    .RoraLink_8B10B_Top_sys_reset_o          (rl_sys_reset),
    .RoraLink_8B10B_Top_user_rx_data_o       (rl_rx_data),
    .RoraLink_8B10B_Top_user_rx_strb_o       (rl_rx_strb),
    .RoraLink_8B10B_Top_user_rx_valid_o      (rl_rx_valid),
    .RoraLink_8B10B_Top_user_rx_last_o       (rl_rx_last),
    .RoraLink_8B10B_Top_crc_pass_fail_n_o    (rl_crc_pass_fail_n),
    .RoraLink_8B10B_Top_crc_valid_o          (rl_crc_valid),
    .RoraLink_8B10B_Top_hard_err_o           (rl_hard_err),
    .RoraLink_8B10B_Top_soft_err_o           (rl_soft_err),
    .RoraLink_8B10B_Top_frame_err_o          (rl_frame_err),
    .RoraLink_8B10B_Top_channel_up_o         (rl_channel_up),
    .RoraLink_8B10B_Top_lane_up_o            (rl_lane_up),
    .RoraLink_8B10B_Top_gt_pcs_rx_clk_o      (rl_rx_clk),
    .RoraLink_8B10B_Top_gt_pll_lock_o        (rl_gt_pll_ok),
    .RoraLink_8B10B_Top_gt_rx_align_link_o   (rl_gt_rx_align_link),
    .RoraLink_8B10B_Top_gt_rx_pma_lock_o     (rl_gt_rx_pma_lock),
    .RoraLink_8B10B_Top_gt_rx_k_lock_o       (rl_gt_rx_k_lock),

    .RoraLink_8B10B_Top_user_clk_i           (rl_rx_clk),
    .RoraLink_8B10B_Top_init_clk_i           (clk),
    .RoraLink_8B10B_Top_reset_i              (global_rst),
    .RoraLink_8B10B_Top_user_pll_locked_i    (rl_gt_pll_ok),
    .RoraLink_8B10B_Top_gt_reset_i           (global_rst),
    .RoraLink_8B10B_Top_gt_pcs_rx_reset_i    (global_rst)
);

// --------------------------------------------------------------------------
// HDMI timing generator
// --------------------------------------------------------------------------
wire        hdmi_hs_raw;
wire        hdmi_vs_raw;
wire        hdmi_de_raw;
wire [11:0] hdmi_x;
wire [10:0] hdmi_y;
wire        hdmi_frame_start;

hdmi_720p_timing u_hdmi_timing (
    .pixel_clk   (pixel_clk),
    .rst_n       (pixel_rst_n),
    .hs          (hdmi_hs_raw),
    .vs          (hdmi_vs_raw),
    .de          (hdmi_de_raw),
    .x           (hdmi_x),
    .y           (hdmi_y),
    .frame_start (hdmi_frame_start)
);

// --------------------------------------------------------------------------
// RoraLink RX -> DDR writer -> DDR reader -> HDMI bridge
// --------------------------------------------------------------------------
wire [31:0] fb_pixel_word;
wire        fb_first_frame_written;
wire        fb_display_started;
wire        fb_error;
wire        fb_underflow_seen;
wire [3:0]  fb_state;
wire [15:0] fb_wr_burst_idx;
wire [15:0] fb_rd_burst_idx;
wire [15:0] fb_rd_fifo_wr_count;
wire [15:0] fb_rd_fifo_rd_count;
wire [15:0] fb_wr_fifo_rd_count;
wire [31:0] rx_packet_cnt;
wire [31:0] rx_crc_pass_cnt;
wire [15:0] rx_good_seg_cnt;
wire        rx_test_pass;
wire        rx_any_err_seen;
wire [11:0] rx_word_idx_dbg;
wire [10:0] rx_cur_line_id_dbg;
wire [4:0]  rx_cur_seg_id_dbg;
wire        rx_capture_active_dbg;
wire        rx_payload_accept_dbg;
wire        rx_wr_fifo_wren_dbg;
wire        rx_header_err_seen_dbg;
wire        rx_last_err_seen_dbg;
wire        rx_crc_err_seen_dbg;
wire        rx_overrun_err_seen_dbg;
wire [15:0] fb_wr_fifo_wr_count;

roralink_video_to_ddr_hdmi_b2 #(
    .AXI_ADDR_WIDTH (29),
    .AXI_DATA_WIDTH (256),
    .AXI_ID_WIDTH   (4),
    .H_RES          (1280),
    .V_RES          (720),
    .BURST_BEATS    (64)
) u_b2_bridge (
    .axi_clk              (clk_out),
    .axi_rst              (ddr_rst | !init_calib_complete),
    .pixel_clk            (pixel_clk),
    .pixel_rst            (!pixel_rst_n),
    .display_de           (hdmi_de_raw),
    .display_frame_start  (hdmi_frame_start),
    .pixel_word           (fb_pixel_word),

    .rx_clk               (rl_rx_clk),
    .rx_rst_async         (global_rst | rl_sys_reset | rl_link_reset | ~rl_gt_pll_ok),
    .rx_channel_up        (rl_channel_up),
    .rx_user_data         (rl_rx_data),
    .rx_user_strb         (rl_rx_strb),
    .rx_user_valid        (rl_rx_valid),
    .rx_user_last         (rl_rx_last),
    .rx_crc_valid         (rl_crc_valid),
    .rx_crc_pass_fail_n   (rl_crc_pass_fail_n),
    .rx_hard_err          (rl_hard_err),
    .rx_soft_err          (rl_soft_err),
    .rx_frame_err         (rl_frame_err),

    .m_axi_awvalid        (s_axi_awvalid),
    .m_axi_awready        (s_axi_awready),
    .m_axi_awid           (s_axi_awid),
    .m_axi_awaddr         (s_axi_awaddr),
    .m_axi_awlen          (s_axi_awlen),
    .m_axi_awsize         (s_axi_awsize),
    .m_axi_awburst        (s_axi_awburst),
    .m_axi_wvalid         (s_axi_wvalid),
    .m_axi_wready         (s_axi_wready),
    .m_axi_wdata          (s_axi_wdata),
    .m_axi_wstrb          (s_axi_wstrb),
    .m_axi_wlast          (s_axi_wlast),
    .m_axi_bvalid         (s_axi_bvalid),
    .m_axi_bready         (s_axi_bready),
    .m_axi_bresp          (s_axi_bresp),
    .m_axi_bid            (s_axi_bid),
    .m_axi_arvalid        (s_axi_arvalid),
    .m_axi_arready        (s_axi_arready),
    .m_axi_arid           (s_axi_arid),
    .m_axi_araddr         (s_axi_araddr),
    .m_axi_arlen          (s_axi_arlen),
    .m_axi_arsize         (s_axi_arsize),
    .m_axi_arburst        (s_axi_arburst),
    .m_axi_rvalid         (s_axi_rvalid),
    .m_axi_rready         (s_axi_rready),
    .m_axi_rdata          (s_axi_rdata),
    .m_axi_rresp          (s_axi_rresp),
    .m_axi_rid            (s_axi_rid),
    .m_axi_rlast          (s_axi_rlast),

    .first_frame_written  (fb_first_frame_written),
    .display_started      (fb_display_started),
    .error_seen           (fb_error),
    .underflow_seen       (fb_underflow_seen),
    .state_dbg            (fb_state),
    .wr_burst_idx_dbg     (fb_wr_burst_idx),
    .rd_burst_idx_dbg     (fb_rd_burst_idx),
    .rd_fifo_wr_count_dbg (fb_rd_fifo_wr_count),
    .rd_fifo_rd_count_dbg (fb_rd_fifo_rd_count),
    .wr_fifo_wr_count_dbg (fb_wr_fifo_wr_count),
    .wr_fifo_rd_count_dbg (fb_wr_fifo_rd_count),
    .rx_packet_cnt_dbg    (rx_packet_cnt),
    .rx_crc_pass_cnt_dbg  (rx_crc_pass_cnt),
    .rx_good_seg_cnt_dbg  (rx_good_seg_cnt),
    .rx_test_pass         (rx_test_pass),
    .rx_any_err_seen      (rx_any_err_seen),
    .rx_word_idx_dbg      (rx_word_idx_dbg),
    .rx_cur_line_id_dbg   (rx_cur_line_id_dbg),
    .rx_cur_seg_id_dbg    (rx_cur_seg_id_dbg),
    .rx_capture_active_dbg(rx_capture_active_dbg),
    .rx_payload_accept_dbg(rx_payload_accept_dbg),
    .rx_wr_fifo_wren_dbg  (rx_wr_fifo_wren_dbg),
    .rx_header_err_seen_dbg(rx_header_err_seen_dbg),
    .rx_last_err_seen_dbg (rx_last_err_seen_dbg),
    .rx_crc_err_seen_dbg  (rx_crc_err_seen_dbg),
    .rx_overrun_err_seen_dbg(rx_overrun_err_seen_dbg)
);

// --------------------------------------------------------------------------
// HDMI output data selection
//   Before DDR display starts: white screen.
//   Active underflow is forced red inside bridge pixel_word.
// --------------------------------------------------------------------------
reg        hdmi_hs_d;
reg        hdmi_vs_d;
reg        hdmi_de_d;
reg [23:0] hdmi_rgb_d;

wire [23:0] ddr_rgb = fb_pixel_word[23:0];
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

// --------------------------------------------------------------------------
// ADV7513 I2C init, copied from verified GW60K_cb flow
// --------------------------------------------------------------------------
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

// --------------------------------------------------------------------------
// Active-low LEDs
//   LED0 on: DDR init + RoraLink channel up
//   LED1 on: received packets OK + first DDR frame written + HDMI display started
// --------------------------------------------------------------------------
reg [25:0] led_cnt;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        led_cnt <= 26'd0;
    else
        led_cnt <= led_cnt + 1'b1;
end

wire link_ok    = init_calib_complete && rl_channel_up && rl_lane_up;
wire video_ok   = rx_test_pass && fb_first_frame_written && fb_display_started && !fb_error && !fb_underflow_seen;
wire any_bad    = fb_error || fb_underflow_seen || rx_any_err_seen;

assign O_led[0] = link_ok ? 1'b0 : led_cnt[25];
assign O_led[1] = any_bad ? led_cnt[22] : (video_ok ? 1'b0 : led_cnt[25]);

// --------------------------------------------------------------------------
// ILA search prefix: ila60b2_
// Kept signals are intentionally trimmed for B2_fix2 debug.
// Suggested GAO clocks:
//   RX-domain  : rl_rx_clk   -> ila60b2_rl_*, ila60b2_rx_*
//   AXI-domain : clk_out     -> ila60b2_fb_*, ila60b2_*fifo*, ila60b2_bresp/rresp
//   HDMI-domain: pixel_clk   -> ila60b2_hdmi_*, ila60b2_display_started
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila60b2_top_version             = 32'h60B2_7202;

// RoraLink RX-domain essentials
(* keep = "true" *) wire        ila60b2_rl_channel_up           = rl_channel_up;
(* keep = "true" *) wire        ila60b2_rl_lane_up              = rl_lane_up;
(* keep = "true" *) wire        ila60b2_rl_pma_lock             = rl_gt_rx_pma_lock;
(* keep = "true" *) wire        ila60b2_rl_k_lock               = rl_gt_rx_k_lock;
(* keep = "true" *) wire        ila60b2_rl_align_link           = rl_gt_rx_align_link;
(* keep = "true" *) wire        ila60b2_rl_rx_valid             = rl_rx_valid;
(* keep = "true" *) wire        ila60b2_rl_rx_last              = rl_rx_last;
(* keep = "true" *) wire [31:0] ila60b2_rl_rx_data              = rl_rx_data;
(* keep = "true" *) wire        ila60b2_rl_crc_valid            = rl_crc_valid;
(* keep = "true" *) wire        ila60b2_rl_crc_pass_fail_n      = rl_crc_pass_fail_n;

// RX parser / DDR-write source state
(* keep = "true" *) wire [31:0] ila60b2_rx_packet_cnt           = rx_packet_cnt;
(* keep = "true" *) wire [31:0] ila60b2_rx_crc_pass_cnt         = rx_crc_pass_cnt;
(* keep = "true" *) wire [15:0] ila60b2_rx_good_seg_cnt         = rx_good_seg_cnt;
(* keep = "true" *) wire        ila60b2_rx_test_pass            = rx_test_pass;
(* keep = "true" *) wire        ila60b2_rx_any_err_seen         = rx_any_err_seen;
(* keep = "true" *) wire [11:0] ila60b2_rx_word_idx             = rx_word_idx_dbg;
(* keep = "true" *) wire [10:0] ila60b2_rx_line_id              = rx_cur_line_id_dbg;
(* keep = "true" *) wire [4:0]  ila60b2_rx_seg_id               = rx_cur_seg_id_dbg;
(* keep = "true" *) wire        ila60b2_rx_capture_active       = rx_capture_active_dbg;
(* keep = "true" *) wire        ila60b2_rx_payload_accept       = rx_payload_accept_dbg;
(* keep = "true" *) wire        ila60b2_rx_wr_fifo_wren         = rx_wr_fifo_wren_dbg;
(* keep = "true" *) wire        ila60b2_rx_header_err_seen      = rx_header_err_seen_dbg;
(* keep = "true" *) wire        ila60b2_rx_last_err_seen        = rx_last_err_seen_dbg;
(* keep = "true" *) wire        ila60b2_rx_crc_err_seen         = rx_crc_err_seen_dbg;
(* keep = "true" *) wire        ila60b2_rx_overrun_err_seen     = rx_overrun_err_seen_dbg;

// AXI/DDR framebuffer state
(* keep = "true" *) wire        ila60b2_init_calib_complete     = init_calib_complete;
(* keep = "true" *) wire        ila60b2_first_frame_written     = fb_first_frame_written;
(* keep = "true" *) wire        ila60b2_fb_error                = fb_error;
(* keep = "true" *) wire [3:0]  ila60b2_fb_state                = fb_state;
(* keep = "true" *) wire [15:0] ila60b2_wr_burst_idx            = fb_wr_burst_idx;
(* keep = "true" *) wire [15:0] ila60b2_rd_burst_idx            = fb_rd_burst_idx;
(* keep = "true" *) wire [15:0] ila60b2_wr_fifo_wr_count        = fb_wr_fifo_wr_count;
(* keep = "true" *) wire [15:0] ila60b2_wr_fifo_rd_count        = fb_wr_fifo_rd_count;
(* keep = "true" *) wire [15:0] ila60b2_rd_fifo_wr_count        = fb_rd_fifo_wr_count;
(* keep = "true" *) wire [15:0] ila60b2_rd_fifo_rd_count        = fb_rd_fifo_rd_count;
(* keep = "true" *) wire [1:0]  ila60b2_bresp                   = s_axi_bresp;
(* keep = "true" *) wire [1:0]  ila60b2_rresp                   = s_axi_rresp;

// HDMI output state
(* keep = "true" *) wire        ila60b2_display_started         = fb_display_started;
(* keep = "true" *) wire        ila60b2_underflow_seen          = fb_underflow_seen;
(* keep = "true" *) wire        ila60b2_hdmi_de                 = hdmi_de_raw;
(* keep = "true" *) wire [11:0] ila60b2_hdmi_x                  = hdmi_x;
(* keep = "true" *) wire [10:0] ila60b2_hdmi_y                  = hdmi_y;
(* keep = "true" *) wire [23:0] ila60b2_hdmi_rgb                = hdmi_rgb_d;

endmodule

// ============================================================================
// 720p60 timing generator
// ============================================================================
module hdmi_720p_timing (
    input  wire        pixel_clk,
    input  wire        rst_n,
    output reg         hs,
    output reg         vs,
    output reg         de,
    output reg [11:0]  x,
    output reg [10:0]  y,
    output wire        frame_start
);
    localparam [15:0] H_TOTAL  = 16'd1650;
    localparam [15:0] H_SYNC   = 16'd40;
    localparam [15:0] H_BPORCH = 16'd220;
    localparam [15:0] H_RES    = 16'd1280;
    localparam [15:0] V_TOTAL  = 16'd750;
    localparam [15:0] V_SYNC   = 16'd5;
    localparam [15:0] V_BPORCH = 16'd20;
    localparam [15:0] V_RES    = 16'd720;
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


// ============================================================================
// RoraLink segmented video RX -> DDR AXI writer/reader bridge
// B2_fix2 720p policy:
//   - Keep the proven DDR read/HDMI path from A2-1.
//   - Use a lenient depacketizer for B2: header-driven capture, no payload
//     checker in the write path.
//   - Start DDR capture only from SOF: line_id=0, segment_id=0.
//   - Do not let startup hard/frame error or payload checker block DDR output.
// ============================================================================

module roralink_video_to_ddr_hdmi_b2 #(
    parameter integer AXI_ADDR_WIDTH = 29,
    parameter integer AXI_DATA_WIDTH = 256,
    parameter integer AXI_ID_WIDTH   = 4,
    parameter integer H_RES          = 1280,
    parameter integer V_RES          = 720,
    parameter integer BURST_BEATS    = 64
)(
    input  wire                         axi_clk,
    input  wire                         axi_rst,
    input  wire                         pixel_clk,
    input  wire                         pixel_rst,
    input  wire                         display_de,
    input  wire                         display_frame_start,
    output wire [31:0]                  pixel_word,

    input  wire                         rx_clk,
    input  wire                         rx_rst_async,
    input  wire                         rx_channel_up,
    input  wire [31:0]                  rx_user_data,
    input  wire [3:0]                   rx_user_strb,
    input  wire                         rx_user_valid,
    input  wire                         rx_user_last,
    input  wire                         rx_crc_valid,
    input  wire                         rx_crc_pass_fail_n,
    input  wire                         rx_hard_err,
    input  wire                         rx_soft_err,
    input  wire                         rx_frame_err,

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

    output reg                          first_frame_written,
    output reg                          display_started,
    output reg                          error_seen,
    output reg                          underflow_seen,
    output reg [3:0]                    state_dbg,
    output reg [15:0]                   wr_burst_idx_dbg,
    output reg [15:0]                   rd_burst_idx_dbg,
    output wire [15:0]                  rd_fifo_wr_count_dbg,
    output wire [15:0]                  rd_fifo_rd_count_dbg,
    output wire [15:0]                  wr_fifo_wr_count_dbg,
    output wire [15:0]                  wr_fifo_rd_count_dbg,
    output reg [31:0]                   rx_packet_cnt_dbg,
    output reg [31:0]                   rx_crc_pass_cnt_dbg,
    output reg [15:0]                   rx_good_seg_cnt_dbg,
    output reg                          rx_test_pass,
    output wire                         rx_any_err_seen,
    output wire [11:0]                  rx_word_idx_dbg,
    output wire [10:0]                  rx_cur_line_id_dbg,
    output wire [4:0]                   rx_cur_seg_id_dbg,
    output wire                         rx_capture_active_dbg,
    output wire                         rx_payload_accept_dbg,
    output wire                         rx_wr_fifo_wren_dbg,
    output wire                         rx_header_err_seen_dbg,
    output wire                         rx_last_err_seen_dbg,
    output wire                         rx_crc_err_seen_dbg,
    output wire                         rx_overrun_err_seen_dbg
);
    localparam [31:0] MAGIC            = 32'hA55A_6002;
    localparam [7:0]  FORMAT_RGB888_32 = 8'h01;
    localparam [11:0] H12              = 12'd1280;
    localparam [11:0] V12              = 12'd720;
    localparam [10:0] LAST_LINE        = 11'd719;
    localparam [4:0]  LAST_SEG         = 5'd4;
    localparam [11:0] SEG_PIXELS_FULL  = 12'd256;
    localparam [11:0] SEG_PIXELS_LAST  = 12'd256;
    localparam [11:0] HEADER_WORDS     = 12'd4;
    localparam [15:0] STABLE_DELAY_MAX = 16'h3FFF;
    localparam [15:0] PASS_SEG_TH      = 16'd512;

    // RX reset and channel-up stable gate.
    reg [3:0] rx_rst_shr = 4'hF;
    always @(posedge rx_clk or posedge rx_rst_async) begin
        if (rx_rst_async)
            rx_rst_shr <= 4'hF;
        else
            rx_rst_shr <= {rx_rst_shr[2:0], 1'b0};
    end
    wire rx_rst_base = rx_rst_shr[3];

    reg [15:0] stable_cnt = 16'd0;
    reg        check_enable = 1'b0;
    always @(posedge rx_clk or posedge rx_rst_base) begin
        if (rx_rst_base) begin
            stable_cnt   <= 16'd0;
            check_enable <= 1'b0;
        end else begin
            if (!rx_channel_up) begin
                stable_cnt   <= 16'd0;
                check_enable <= 1'b0;
            end else if (!check_enable) begin
                if (stable_cnt == STABLE_DELAY_MAX)
                    check_enable <= 1'b1;
                else
                    stable_cnt <= stable_cnt + 1'b1;
            end
        end
    end

    wire rx_active = check_enable && rx_channel_up && !rx_rst_base;
    wire rx_fire   = rx_active && rx_user_valid;

    function [11:0] segment_x_base;
        input [4:0] s;
        begin
            segment_x_base = {s[3:0], 8'h00};
        end
    endfunction

    function [11:0] segment_payload_words;
        input [4:0] s;
        begin
            segment_payload_words = (s == LAST_SEG) ? SEG_PIXELS_LAST : SEG_PIXELS_FULL;
        end
    endfunction

    // Minimal header fields.
    wire [15:0] w1_frame = rx_user_data[31:16];
    wire [10:0] w1_line  = rx_user_data[15:5];
    wire [4:0]  w1_seg   = rx_user_data[4:0];
    wire [7:0]  w2_format = rx_user_data[31:24];
    wire [11:0] w2_x_base = rx_user_data[23:12];
    wire [11:0] w2_payload_words = rx_user_data[11:0];
    wire [11:0] w3_width = rx_user_data[23:12];
    wire [11:0] w3_height = rx_user_data[11:0];

    wire        wr_fifo_full;
    wire        wr_fifo_empty;
    wire        wr_fifo_aempty;
    wire        wr_fifo_afull;
    wire [255:0] wr_fifo_q;
    wire [11:0] wr_fifo_wnum;
    wire [11:0] wr_fifo_rnum;
    reg         wr_fifo_rden;

    // RX depacketizer.
    reg        rx_aligned       = 1'b0;
    reg        capture_active   = 1'b0;
    reg        capture_this_pkt = 1'b0;
    reg        packet_magic_ok  = 1'b0;
    reg        packet_header_ok = 1'b0;
    reg [11:0] word_idx         = 12'd0;
    reg [15:0] cur_frame_id     = 16'd0;
    reg [10:0] cur_line_id      = 11'd0;
    reg [4:0]  cur_seg_id       = 5'd0;
    reg [11:0] cur_x_base       = 12'd0;
    reg [11:0] cur_payload_words= 12'd0;
    reg [255:0] rx_pack_reg     = 256'd0;
    reg [255:0] rx_pack_next;
    reg [255:0] rx_fifo_wdata   = 256'd0;
    reg         rx_fifo_wren    = 1'b0;

    reg [31:0] rx_packet_cnt    = 32'd0;
    reg [31:0] rx_crc_pass_cnt  = 32'd0;
    reg [15:0] rx_good_seg_cnt  = 16'd0;
    reg [31:0] rx_valid_cnt     = 32'd0;

    reg header_err_seen  = 1'b0;
    reg last_err_seen    = 1'b0;
    reg crc_err_seen     = 1'b0;
    reg strb_err_seen    = 1'b0;
    reg overrun_err_seen = 1'b0;

    wire [11:0] payload_pos    = word_idx - HEADER_WORDS;
    wire        payload_region = (word_idx >= HEADER_WORDS) &&
                                 (word_idx < (HEADER_WORDS + cur_payload_words));
    wire        exp_last_now   = rx_aligned && (word_idx == (HEADER_WORDS + cur_payload_words - 12'd1));
    wire        payload_accept = rx_fire && payload_region && capture_this_pkt && packet_header_ok;

    assign rx_any_err_seen = header_err_seen | last_err_seen | crc_err_seen | strb_err_seen | overrun_err_seen;

    always @* begin
        rx_pack_next = rx_pack_reg;
        case (payload_pos[2:0])
            3'd0: rx_pack_next[31:0]    = rx_user_data;
            3'd1: rx_pack_next[63:32]   = rx_user_data;
            3'd2: rx_pack_next[95:64]   = rx_user_data;
            3'd3: rx_pack_next[127:96]  = rx_user_data;
            3'd4: rx_pack_next[159:128] = rx_user_data;
            3'd5: rx_pack_next[191:160] = rx_user_data;
            3'd6: rx_pack_next[223:192] = rx_user_data;
            3'd7: rx_pack_next[255:224] = rx_user_data;
        endcase
    end

    always @(posedge rx_clk or posedge rx_rst_base) begin
        if (rx_rst_base) begin
            rx_aligned          <= 1'b0;
            capture_active      <= 1'b0;
            capture_this_pkt    <= 1'b0;
            packet_magic_ok     <= 1'b0;
            packet_header_ok    <= 1'b0;
            word_idx            <= 12'd0;
            cur_frame_id        <= 16'd0;
            cur_line_id         <= 11'd0;
            cur_seg_id          <= 5'd0;
            cur_x_base          <= 12'd0;
            cur_payload_words   <= 12'd0;
            rx_pack_reg         <= 256'd0;
            rx_fifo_wdata       <= 256'd0;
            rx_fifo_wren        <= 1'b0;
            rx_packet_cnt       <= 32'd0;
            rx_crc_pass_cnt     <= 32'd0;
            rx_good_seg_cnt     <= 16'd0;
            rx_valid_cnt        <= 32'd0;
            rx_packet_cnt_dbg   <= 32'd0;
            rx_crc_pass_cnt_dbg <= 32'd0;
            rx_good_seg_cnt_dbg <= 16'd0;
            rx_test_pass        <= 1'b0;
            header_err_seen     <= 1'b0;
            last_err_seen       <= 1'b0;
            crc_err_seen        <= 1'b0;
            strb_err_seen       <= 1'b0;
            overrun_err_seen    <= 1'b0;
        end else begin
            rx_fifo_wren <= 1'b0;

            if (!rx_active) begin
                rx_aligned       <= 1'b0;
                capture_active   <= 1'b0;
                capture_this_pkt <= 1'b0;
                word_idx         <= 12'd0;
                rx_test_pass     <= 1'b0;
            end else begin
                if (rx_crc_valid) begin
                    if (rx_crc_pass_fail_n) begin
                        rx_crc_pass_cnt     <= rx_crc_pass_cnt + 1'b1;
                        rx_crc_pass_cnt_dbg <= rx_crc_pass_cnt + 1'b1;
                    end else begin
                        // Count it, but do not stop capture in B2_fix2.
                        crc_err_seen <= 1'b1;
                    end
                end

                if (rx_fire) begin
                    rx_valid_cnt <= rx_valid_cnt + 1'b1;

                    if (!rx_aligned) begin
                        // Throw away the current partial frame after startup.
                        if (rx_user_last) begin
                            rx_aligned <= 1'b1;
                            word_idx   <= 12'd0;
                        end
                    end else begin
                        if (word_idx == 12'd0) begin
                            packet_magic_ok  <= (rx_user_data == MAGIC);
                            packet_header_ok <= (rx_user_data == MAGIC);
                            capture_this_pkt <= capture_active;
                            rx_pack_reg      <= 256'd0;
                            if (rx_user_data != MAGIC)
                                header_err_seen <= 1'b1;
                        end

                        if (word_idx == 12'd1) begin
                            cur_frame_id <= w1_frame;
                            cur_line_id  <= w1_line;
                            cur_seg_id   <= w1_seg;

                            if ((w1_line > LAST_LINE) || (w1_seg > LAST_SEG)) begin
                                packet_header_ok <= 1'b0;
                                header_err_seen  <= 1'b1;
                            end

                            // Start DDR capture from a clean frame boundary.
                            // Once started, continue writing subsequent packets.
                            if (packet_magic_ok && (w1_line == 11'd0) && (w1_seg == 5'd0)) begin
                                capture_active   <= 1'b1;
                                capture_this_pkt <= 1'b1;
                                rx_pack_reg      <= 256'd0;
                            end else begin
                                capture_this_pkt <= capture_active && packet_magic_ok;
                            end
                        end

                        if (word_idx == 12'd2) begin
                            cur_x_base        <= w2_x_base;
                            cur_payload_words <= w2_payload_words;
                            if ((w2_format != FORMAT_RGB888_32) ||
                                (w2_x_base != segment_x_base(cur_seg_id)) ||
                                (w2_payload_words != segment_payload_words(cur_seg_id))) begin
                                packet_header_ok <= 1'b0;
                                header_err_seen  <= 1'b1;
                            end
                        end

                        if (word_idx == 12'd3) begin
                            if ((w3_width != H12) || (w3_height != V12)) begin
                                packet_header_ok <= 1'b0;
                                header_err_seen  <= 1'b1;
                            end
                        end

                        if (rx_user_strb != 4'hF)
                            strb_err_seen <= 1'b1;

                        // B2_fix2: no payload comparison. If header says this is a
                        // payload word, pack it and feed the DDR write FIFO.
                        if (payload_region) begin
                            if (payload_accept) begin
                                rx_pack_reg <= rx_pack_next;
                                if (payload_pos[2:0] == 3'd7) begin
                                    if (wr_fifo_full) begin
                                        overrun_err_seen <= 1'b1;
                                        capture_active   <= 1'b0;
                                    end else begin
                                        rx_fifo_wdata <= rx_pack_next;
                                        rx_fifo_wren  <= 1'b1;
                                    end
                                end
                            end
                        end

                        if (rx_user_last) begin
                            rx_packet_cnt     <= rx_packet_cnt + 1'b1;
                            rx_packet_cnt_dbg <= rx_packet_cnt + 1'b1;

                            if (!exp_last_now)
                                last_err_seen <= 1'b1;

                            if (packet_header_ok && exp_last_now) begin
                                if (rx_good_seg_cnt != 16'hFFFF)
                                    rx_good_seg_cnt <= rx_good_seg_cnt + 1'b1;
                                rx_good_seg_cnt_dbg <= rx_good_seg_cnt + 1'b1;
                            end

                            word_idx <= 12'd0;
                        end else begin
                            if (exp_last_now)
                                last_err_seen <= 1'b1;
                            word_idx <= word_idx + 1'b1;
                        end
                    end
                end

                if ((rx_good_seg_cnt >= PASS_SEG_TH) && !overrun_err_seen && !strb_err_seen && !last_err_seen && !crc_err_seen)
                    rx_test_pass <= 1'b1;
            end
        end
    end

    wire wr_fifo_reset = rx_rst_base | !check_enable;

    Video_WR_FIFO_256 u_video_wr_fifo_256 (
        .Data         (rx_fifo_wdata),
        .Reset        (wr_fifo_reset),
        .WrClk        (rx_clk),
        .RdClk        (axi_clk),
        .WrEn         (rx_fifo_wren),
        .RdEn         (wr_fifo_rden),
        .Wnum         (wr_fifo_wnum),
        .Rnum         (wr_fifo_rnum),
        .Almost_Empty (wr_fifo_aempty),
        .Almost_Full  (wr_fifo_afull),
        .Q            (wr_fifo_q),
        .Empty        (wr_fifo_empty),
        .Full         (wr_fifo_full)
    );

    assign rx_word_idx_dbg          = word_idx;
    assign rx_cur_line_id_dbg       = cur_line_id;
    assign rx_cur_seg_id_dbg        = cur_seg_id;
    assign rx_capture_active_dbg    = capture_active;
    assign rx_payload_accept_dbg    = payload_accept;
    assign rx_wr_fifo_wren_dbg      = rx_fifo_wren;
    assign rx_header_err_seen_dbg   = header_err_seen;
    assign rx_last_err_seen_dbg     = last_err_seen;
    assign rx_crc_err_seen_dbg      = crc_err_seen;
    assign rx_overrun_err_seen_dbg  = overrun_err_seen;

    // DDR read-side FIFO: 256bit clk_out -> 32bit pixel_clk.
    wire        rd_fifo_rst = axi_rst | !first_frame_written;
    reg         rd_fifo_wren;
    wire [31:0] rd_fifo_q;
    wire        rd_fifo_empty;
    wire        rd_fifo_full;
    wire        rd_fifo_aempty;
    wire        rd_fifo_afull;
    wire [10:0] rd_fifo_wnum;
    wire [13:0] rd_fifo_rnum;
    wire        rd_fifo_rden;

    Video_FIFO_256to32 u_video_fifo_256to32 (
        .Data         (m_axi_rdata),
        .Reset        (rd_fifo_rst),
        .WrClk        (axi_clk),
        .RdClk        (pixel_clk),
        .WrEn         (rd_fifo_wren),
        .RdEn         (rd_fifo_rden),
        .Wnum         (rd_fifo_wnum),
        .Rnum         (rd_fifo_rnum),
        .Almost_Empty (rd_fifo_aempty),
        .Almost_Full  (rd_fifo_afull),
        .Q            (rd_fifo_q),
        .Empty        (rd_fifo_empty),
        .Full         (rd_fifo_full)
    );

    localparam integer PIXELS_PER_BEAT  = AXI_DATA_WIDTH / 32;
    localparam integer FRAME_PIXELS     = H_RES * V_RES;
    localparam integer FRAME_BEATS      = FRAME_PIXELS / PIXELS_PER_BEAT;
    localparam integer TOTAL_BURSTS     = FRAME_BEATS / BURST_BEATS;
    localparam [7:0]   AXI_LEN          = BURST_BEATS - 1;
    localparam [13:0]  FIFO_START_LEVEL = 14'd4096;
    localparam [10:0]  RD_FIFO_WR_SAFE_LEVEL = 11'd832;

    // DEBUG: set to 1'b1 to freeze after the first complete frame.
    // Default 0 for normal continuous 720p video transmission.
    localparam DEBUG_FREEZE_AFTER_FIRST_FRAME = 1'b0;

    localparam [3:0]
        ST_IDLE  = 4'd0,
        ST_WR_AW = 4'd1,
        ST_WR_W  = 4'd2,
        ST_WR_B  = 4'd3,
        ST_RD_AR = 4'd4,
        ST_RD_R  = 4'd5,
        ST_ERROR = 4'd15;

    assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_awlen   = AXI_LEN;
    assign m_axi_arlen   = AXI_LEN;
    assign m_axi_awsize  = 3'b101;
    assign m_axi_arsize  = 3'b101;
    assign m_axi_awburst = 2'b01;
    assign m_axi_arburst = 2'b01;
    assign m_axi_wstrb   = {AXI_DATA_WIDTH/8{1'b1}};

    reg [3:0]  state;
    reg [15:0] wr_burst_idx;
    reg [15:0] rd_burst_idx;
    reg [7:0]  beat_idx;

    assign m_axi_awaddr = {{(AXI_ADDR_WIDTH-27){1'b0}}, wr_burst_idx, 11'b0};
    assign m_axi_araddr = {{(AXI_ADDR_WIDTH-27){1'b0}}, rd_burst_idx, 11'b0};
    assign m_axi_wdata  = wr_fifo_q;
    assign m_axi_wlast  = m_axi_wvalid && (beat_idx == AXI_LEN);

    localparam [11:0] WR_BURST_LEVEL = BURST_BEATS;
    wire wr_burst_available = (wr_fifo_rnum >= WR_BURST_LEVEL);
    wire rd_fifo_has_space  = (rd_fifo_wnum <= RD_FIFO_WR_SAFE_LEVEL);

    assign rd_fifo_wr_count_dbg = {5'd0, rd_fifo_wnum};
    assign rd_fifo_rd_count_dbg = {2'd0, rd_fifo_rnum};
    assign wr_fifo_wr_count_dbg = {4'd0, wr_fifo_wnum};
    assign wr_fifo_rd_count_dbg = {4'd0, wr_fifo_rnum};

    assign rd_fifo_rden = display_started && display_de && !rd_fifo_empty;
    assign pixel_word   = (display_started && display_de && rd_fifo_empty) ? 32'h00FF_0000 : rd_fifo_q;

    // Pixel-domain display start and underflow detect.
    reg first_frame_written_p1;
    reg first_frame_written_p2;
    always @(posedge pixel_clk or posedge pixel_rst) begin
        if (pixel_rst) begin
            first_frame_written_p1 <= 1'b0;
            first_frame_written_p2 <= 1'b0;
            display_started        <= 1'b0;
            underflow_seen         <= 1'b0;
        end else begin
            first_frame_written_p1 <= first_frame_written;
            first_frame_written_p2 <= first_frame_written_p1;
            if (!display_started && first_frame_written_p2 && (rd_fifo_rnum >= FIFO_START_LEVEL) && display_frame_start)
                display_started <= 1'b1;
            if (display_started && display_de && rd_fifo_empty)
                underflow_seen <= 1'b1;
        end
    end

    // AXI write/read FSM.
    always @(posedge axi_clk) begin
        if (axi_rst) begin
            state               <= ST_IDLE;
            m_axi_awvalid       <= 1'b0;
            m_axi_wvalid        <= 1'b0;
            m_axi_bready        <= 1'b0;
            m_axi_arvalid       <= 1'b0;
            m_axi_rready        <= 1'b0;
            wr_fifo_rden        <= 1'b0;
            rd_fifo_wren        <= 1'b0;
            first_frame_written <= 1'b0;
            error_seen          <= 1'b0;
            wr_burst_idx        <= 16'd0;
            rd_burst_idx        <= 16'd0;
            beat_idx            <= 8'd0;
            state_dbg           <= ST_IDLE;
            wr_burst_idx_dbg    <= 16'd0;
            rd_burst_idx_dbg    <= 16'd0;
        end else begin
            wr_fifo_rden     <= 1'b0;
            rd_fifo_wren     <= 1'b0;
            state_dbg        <= state;
            wr_burst_idx_dbg <= wr_burst_idx;
            rd_burst_idx_dbg <= rd_burst_idx;

            case (state)
                ST_IDLE: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;

                    if (!first_frame_written) begin
                        if (wr_burst_available) begin
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_WR_AW;
                        end
                    end else begin
                        if (!DEBUG_FREEZE_AFTER_FIRST_FRAME &&
                            (wr_fifo_rnum >= 12'd1024) && wr_burst_available) begin
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_WR_AW;
                        end else if (rd_fifo_has_space) begin
                            m_axi_arvalid <= 1'b1;
                            state         <= ST_RD_AR;
                        end else if (!DEBUG_FREEZE_AFTER_FIRST_FRAME && wr_burst_available) begin
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_WR_AW;
                        end
                    end
                end

                ST_WR_AW: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b1;
                        beat_idx      <= 8'd0;
                        state         <= ST_WR_W;
                    end
                end

                ST_WR_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        if (wr_fifo_empty) begin
                            error_seen   <= 1'b1;
                            m_axi_wvalid <= 1'b0;
                            state        <= ST_ERROR;
                        end else begin
                            wr_fifo_rden <= 1'b1;
                            if (beat_idx == AXI_LEN) begin
                                m_axi_wvalid <= 1'b0;
                                m_axi_bready <= 1'b1;
                                state        <= ST_WR_B;
                            end else begin
                                beat_idx <= beat_idx + 1'b1;
                            end
                        end
                    end
                end

                ST_WR_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        if (m_axi_bresp != 2'b00) begin
                            error_seen <= 1'b1;
                            state      <= ST_ERROR;
                        end else begin
                            if (wr_burst_idx == TOTAL_BURSTS - 1) begin
                                wr_burst_idx        <= 16'd0;
                                first_frame_written <= 1'b1;
                            end else begin
                                wr_burst_idx <= wr_burst_idx + 1'b1;
                            end
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_RD_AR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        beat_idx      <= 8'd0;
                        state         <= ST_RD_R;
                    end
                end

                ST_RD_R: begin
                    m_axi_rready <= 1'b1;
                    if (m_axi_rvalid) begin
                        if ((m_axi_rresp != 2'b00) || rd_fifo_full) begin
                            error_seen   <= 1'b1;
                            m_axi_rready <= 1'b0;
                            state        <= ST_ERROR;
                        end else begin
                            rd_fifo_wren <= 1'b1;
                            if (m_axi_rlast) begin
                                m_axi_rready <= 1'b0;
                                if (rd_burst_idx == TOTAL_BURSTS - 1)
                                    rd_burst_idx <= 16'd0;
                                else
                                    rd_burst_idx <= rd_burst_idx + 1'b1;
                                state <= ST_IDLE;
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
                    wr_fifo_rden  <= 1'b0;
                    rd_fifo_wren  <= 1'b0;
                    error_seen    <= 1'b1;
                end

                default: state <= ST_ERROR;
            endcase
        end
    end
endmodule
