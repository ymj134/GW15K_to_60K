// ============================================================================
// GW5AT-60K B2-DB-fix2 modular top: 15K RoraLink segmented video RX -> DDR3 -> HDMI
// ----------------------------------------------------------------------------
// First-pass modular split from the verified monolithic 720p B2 top.
// Function is intended to stay unchanged.
//
// Required generated/source modules:
//   SerDes_Top
//   DDR3_Memory_Interface_Top
//   Gowin_PLL_DDR
//   Gowin_PLL
//   pll_mDRP_intf
//   adv7513_iic_init
//   I2C_MASTER_Top
//   Video_FIFO_256to32
//   Video_WR_FIFO_256
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
// Clock/reset wires
// --------------------------------------------------------------------------
wire rst_n_50m;
wire global_rst;
wire pixel_clk;
wire pixel_pll_lock;
wire pixel_rst_n;
wire memory_clk;
wire ddr_pll_lock;
wire pll_stop;

clock_reset_60k u_clock_reset_60k (
    .clk            (clk),
    .rst_n          (rst_n),
    .pll_stop       (pll_stop),
    .rst_n_50m      (rst_n_50m),
    .global_rst     (global_rst),
    .pixel_clk      (pixel_clk),
    .pixel_pll_lock (pixel_pll_lock),
    .pixel_rst_n    (pixel_rst_n),
    .memory_clk     (memory_clk),
    .ddr_pll_lock   (ddr_pll_lock)
);

// --------------------------------------------------------------------------
// DDR3 AXI wires
// --------------------------------------------------------------------------
wire         clk_out;
wire         ddr_rst;
wire         init_calib_complete;

wire         s_axi_awvalid;
wire         s_axi_awready;
wire [3:0]   s_axi_awid;
wire [28:0]  s_axi_awaddr;
wire [7:0]   s_axi_awlen;
wire [2:0]   s_axi_awsize;
wire [1:0]   s_axi_awburst;
wire         s_axi_wvalid;
wire         s_axi_wready;
wire [255:0] s_axi_wdata;
wire [31:0]  s_axi_wstrb;
wire         s_axi_wlast;
wire         s_axi_bvalid;
wire         s_axi_bready;
wire [1:0]   s_axi_bresp;
wire [3:0]   s_axi_bid;
wire         s_axi_arvalid;
wire         s_axi_arready;
wire [3:0]   s_axi_arid;
wire [28:0]  s_axi_araddr;
wire [7:0]   s_axi_arlen;
wire [2:0]   s_axi_arsize;
wire [1:0]   s_axi_arburst;
wire         s_axi_rvalid;
wire         s_axi_rready;
wire [255:0] s_axi_rdata;
wire [1:0]   s_axi_rresp;
wire [3:0]   s_axi_rid;
wire         s_axi_rlast;

ddr3_axi_wrapper_60k u_ddr3_axi_wrapper (
    .clk                 (clk),
    .rst_n               (rst_n_50m),
    .memory_clk          (memory_clk),
    .pll_lock            (ddr_pll_lock),
    .pll_stop            (pll_stop),
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
// RoraLink RX-only IP wrapper
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

roralink_rx_wrapper u_rl_rx_wrapper (
    .init_clk        (clk),
    .global_rst      (global_rst),
    .link_reset      (rl_link_reset),
    .sys_reset       (rl_sys_reset),
    .rx_data         (rl_rx_data),
    .rx_strb         (rl_rx_strb),
    .rx_valid        (rl_rx_valid),
    .rx_last         (rl_rx_last),
    .crc_pass_fail_n (rl_crc_pass_fail_n),
    .crc_valid       (rl_crc_valid),
    .hard_err        (rl_hard_err),
    .soft_err        (rl_soft_err),
    .frame_err       (rl_frame_err),
    .channel_up      (rl_channel_up),
    .lane_up         (rl_lane_up),
    .rx_clk          (rl_rx_clk),
    .gt_pll_ok       (rl_gt_pll_ok),
    .rx_align_link   (rl_gt_rx_align_link),
    .rx_pma_lock     (rl_gt_rx_pma_lock),
    .rx_k_lock       (rl_gt_rx_k_lock)
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
wire [15:0] fb_wr_fifo_wr_count;
wire        db_wr_buf_sel;
wire        db_rd_buf_sel;
wire        db_pending_valid;
wire        db_pending_buf_sel;
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
wire        rx_frame_accept_allowed_dbg;
wire        rx_drop_frame_active_dbg;

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
    .db_wr_buf_sel_dbg    (db_wr_buf_sel),
    .db_rd_buf_sel_dbg    (db_rd_buf_sel),
    .db_pending_valid_dbg (db_pending_valid),
    .db_pending_buf_sel_dbg(db_pending_buf_sel),
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
    .rx_overrun_err_seen_dbg(rx_overrun_err_seen_dbg),
    .rx_frame_accept_allowed_dbg(rx_frame_accept_allowed_dbg),
    .rx_drop_frame_active_dbg(rx_drop_frame_active_dbg)
);

// --------------------------------------------------------------------------
// HDMI output stage
// --------------------------------------------------------------------------
wire [23:0] hdmi_rgb_d;

hdmi_output_stage u_hdmi_output_stage (
    .pixel_clk       (pixel_clk),
    .pixel_rst_n     (pixel_rst_n),
    .hs_raw          (hdmi_hs_raw),
    .vs_raw          (hdmi_vs_raw),
    .de_raw          (hdmi_de_raw),
    .pixel_word      (fb_pixel_word),
    .display_started (fb_display_started),
    .fb_error        (fb_error),
    .adv_clk         (O_adv7513_clk),
    .adv_hs          (O_adv7513_hs),
    .adv_vs          (O_adv7513_vs),
    .adv_de          (O_adv7513_de),
    .adv_data        (O_adv7513_data),
    .rgb_dbg         (hdmi_rgb_d)
);

// --------------------------------------------------------------------------
// ADV7513 I2C initialization
// --------------------------------------------------------------------------
adv7513_iic_wrapper u_adv7513_iic_wrapper (
    .clk       (clk),
    .rst_n_50m (rst_n_50m),
    .scl       (IO_adv7513_scl),
    .sda       (IO_adv7513_sda)
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

wire link_ok  = init_calib_complete && rl_channel_up && rl_lane_up;
wire video_ok = rx_test_pass && fb_first_frame_written && fb_display_started && !fb_error && !fb_underflow_seen;
wire any_bad  = fb_error || fb_underflow_seen || rx_any_err_seen;

assign O_led[0] = link_ok ? 1'b0 : led_cnt[25];
assign O_led[1] = any_bad ? led_cnt[22] : (video_ok ? 1'b0 : led_cnt[25]);

// --------------------------------------------------------------------------
// ILA search prefix: ila60b2_
// Kept signals are intentionally trimmed for B2 debug.
// Suggested GAO clocks:
//   RX-domain  : rl_rx_clk   -> ila60b2_rl_*, ila60b2_rx_*
//   AXI-domain : clk_out     -> ila60b2_fb_*, ila60b2_*fifo*, ila60b2_bresp/rresp
//   HDMI-domain: pixel_clk   -> ila60b2_hdmi_*, ila60b2_display_started
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila60b2_top_version             = 32'h60B3_1201;

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
(* keep = "true" *) wire        ila60b2_rx_frame_accept_allowed = rx_frame_accept_allowed_dbg;
(* keep = "true" *) wire        ila60b2_rx_drop_frame_active    = rx_drop_frame_active_dbg;

// AXI/DDR framebuffer state
(* keep = "true" *) wire        ila60b2_init_calib_complete     = init_calib_complete;
(* keep = "true" *) wire        ila60b2_first_frame_written     = fb_first_frame_written;
(* keep = "true" *) wire        ila60b2_fb_error                = fb_error;
(* keep = "true" *) wire [3:0]  ila60b2_fb_state                = fb_state;
(* keep = "true" *) wire [15:0] ila60b2_wr_burst_idx            = fb_wr_burst_idx;
(* keep = "true" *) wire [15:0] ila60b2_rd_burst_idx            = fb_rd_burst_idx;
(* keep = "true" *) wire        ila60b2_db_wr_buf_sel            = db_wr_buf_sel;
(* keep = "true" *) wire        ila60b2_db_rd_buf_sel            = db_rd_buf_sel;
(* keep = "true" *) wire        ila60b2_db_pending_valid         = db_pending_valid;
(* keep = "true" *) wire        ila60b2_db_pending_buf_sel       = db_pending_buf_sel;
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
