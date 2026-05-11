// ============================================================================
// 15K top - Modular RoraLink TX-only 720p segmented colorbar source
// ----------------------------------------------------------------------------
// Files required:
//   top.v
//   roralink_tx_wrapper.v
//   video_segment_packetizer.v
//   video_colorbar_rgb.v
//   serdes/serdes.v   // generated SerDes_Top / RoraLink TX-only IP
// ============================================================================

`timescale 1ns / 1ps

module top (
    input  wire clk,      // 50MHz board clock, used as RoraLink init clock
    input  wire rst_n,
    output wire led
);

// --------------------------------------------------------------------------
// RoraLink TX interface
// --------------------------------------------------------------------------
wire        rl_sys_reset;
wire        user_tx_ready;
wire        hard_err;
wire        channel_up;
wire        lane_up;
wire        tx_clk;
wire        gt_pll_ok;
wire        tx_rst;

wire [31:0] user_tx_data;
wire [3:0]  user_tx_strb;
wire        user_tx_valid;
wire        user_tx_last;

roralink_tx_wrapper u_rl_tx_wrapper (
    .init_clk      (clk),
    .rst_n         (rst_n),

    .tx_clk        (tx_clk),
    .tx_rst        (tx_rst),
    .rl_sys_reset  (rl_sys_reset),
    .gt_pll_ok     (gt_pll_ok),
    .lane_up       (lane_up),
    .channel_up    (channel_up),
    .hard_err      (hard_err),
    .user_tx_ready (user_tx_ready),

    .user_tx_data  (user_tx_data),
    .user_tx_strb  (user_tx_strb),
    .user_tx_valid (user_tx_valid),
    .user_tx_last  (user_tx_last)
);

// --------------------------------------------------------------------------
// 720p segmented colorbar packetizer
// --------------------------------------------------------------------------
wire        pkt_fire;
wire [1:0]  pkt_state;
wire [15:0] pkt_frame_id;
wire [10:0] pkt_line_id;
wire [4:0]  pkt_segment_id;
wire [11:0] pkt_word_idx;
wire [11:0] pkt_payload_words;
wire [11:0] pkt_x_base;
wire [11:0] pkt_pixel_x;
wire [31:0] pkt_packet_cnt;
wire [31:0] pkt_word_cnt;
wire [11:0] pkt_gap_cnt;
wire [11:0] pkt_gap_target;
wire [15:0] pkt_start_delay;
wire        pkt_start_done;
wire        pkt_fire_seen;
wire        pkt_last_seen;
wire        pkt_ready_seen;
wire        pkt_channel_seen;
wire        pkt_lane_seen;
wire        pkt_hard_err_seen;

video_segment_packetizer #(
    .H_RES              (12'd1280),
    .V_RES              (12'd720),
    .LAST_LINE          (11'd719),
    .LAST_SEG           (5'd4),
    .SEG_PIXELS_FULL    (12'd256),
    .SEG_PIXELS_LAST    (12'd256),
    .LINK_RATE_IS_3G125 (1'b1),
    .COLOR_BAR_WIDTH    (12'd160)
) u_video_segment_packetizer (
    .clk                (tx_clk),
    .rst                (tx_rst),
    .channel_up         (channel_up),
    .lane_up            (lane_up),
    .hard_err           (hard_err),
    .tx_ready           (user_tx_ready),

    .tx_data            (user_tx_data),
    .tx_strb            (user_tx_strb),
    .tx_valid           (user_tx_valid),
    .tx_last            (user_tx_last),

    .tx_fire_dbg        (pkt_fire),
    .tx_state_dbg       (pkt_state),
    .frame_id_dbg       (pkt_frame_id),
    .line_id_dbg        (pkt_line_id),
    .segment_id_dbg     (pkt_segment_id),
    .word_idx_dbg       (pkt_word_idx),
    .payload_words_dbg  (pkt_payload_words),
    .x_base_dbg         (pkt_x_base),
    .pixel_x_dbg        (pkt_pixel_x),
    .packet_cnt_dbg     (pkt_packet_cnt),
    .word_cnt_dbg       (pkt_word_cnt),
    .gap_cnt_dbg        (pkt_gap_cnt),
    .gap_target_dbg     (pkt_gap_target),
    .start_delay_dbg    (pkt_start_delay),
    .start_done_dbg     (pkt_start_done),
    .fire_seen_dbg      (pkt_fire_seen),
    .last_seen_dbg      (pkt_last_seen),
    .ready_seen_dbg     (pkt_ready_seen),
    .channel_seen_dbg   (pkt_channel_seen),
    .lane_seen_dbg      (pkt_lane_seen),
    .hard_err_seen_dbg  (pkt_hard_err_seen)
);

// --------------------------------------------------------------------------
// LED: high-active for the 15K single LED design used in previous tests.
// --------------------------------------------------------------------------
reg [25:0] led_cnt = 26'd0;
always @(posedge tx_clk or posedge tx_rst) begin
    if (tx_rst)
        led_cnt <= 26'd0;
    else
        led_cnt <= led_cnt + 1'b1;
end

assign led = pkt_hard_err_seen ? led_cnt[22] :
             (channel_up && pkt_last_seen) ? 1'b1 : led_cnt[25];

// --------------------------------------------------------------------------
// ILA signals, search prefix: ila15v_tx_
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15v_tx_top_version    = 32'h1507_3001;
(* keep = "true" *) wire        ila15v_tx_gt_pll_ok      = gt_pll_ok;
(* keep = "true" *) wire        ila15v_tx_rl_sys_reset   = rl_sys_reset;
(* keep = "true" *) wire        ila15v_tx_tx_rst         = tx_rst;
(* keep = "true" *) wire        ila15v_tx_lane_up        = lane_up;
(* keep = "true" *) wire        ila15v_tx_channel_up     = channel_up;
(* keep = "true" *) wire        ila15v_tx_user_tx_ready  = user_tx_ready;
(* keep = "true" *) wire        ila15v_tx_user_tx_valid  = user_tx_valid;
(* keep = "true" *) wire        ila15v_tx_user_tx_last   = user_tx_last;
(* keep = "true" *) wire [31:0] ila15v_tx_user_tx_data   = user_tx_data;
(* keep = "true" *) wire [3:0]  ila15v_tx_user_tx_strb   = user_tx_strb;
(* keep = "true" *) wire        ila15v_tx_fire           = pkt_fire;
(* keep = "true" *) wire [1:0]  ila15v_tx_state          = pkt_state;
(* keep = "true" *) wire [15:0] ila15v_tx_frame_id       = pkt_frame_id;
(* keep = "true" *) wire [10:0] ila15v_tx_line_id        = pkt_line_id;
(* keep = "true" *) wire [4:0]  ila15v_tx_segment_id     = pkt_segment_id;
(* keep = "true" *) wire [11:0] ila15v_tx_word_idx       = pkt_word_idx;
(* keep = "true" *) wire [11:0] ila15v_tx_payload_words  = pkt_payload_words;
(* keep = "true" *) wire [11:0] ila15v_tx_x_base         = pkt_x_base;
(* keep = "true" *) wire [11:0] ila15v_tx_pixel_x        = pkt_pixel_x;
(* keep = "true" *) wire [31:0] ila15v_tx_packet_cnt     = pkt_packet_cnt;
(* keep = "true" *) wire [31:0] ila15v_tx_word_cnt       = pkt_word_cnt;
(* keep = "true" *) wire [11:0] ila15v_tx_gap_cnt        = pkt_gap_cnt;
(* keep = "true" *) wire [11:0] ila15v_tx_gap_target     = pkt_gap_target;
(* keep = "true" *) wire [15:0] ila15v_tx_start_delay    = pkt_start_delay;
(* keep = "true" *) wire        ila15v_tx_start_done     = pkt_start_done;
(* keep = "true" *) wire        ila15v_tx_fire_seen      = pkt_fire_seen;
(* keep = "true" *) wire        ila15v_tx_last_seen      = pkt_last_seen;
(* keep = "true" *) wire        ila15v_tx_ready_seen     = pkt_ready_seen;
(* keep = "true" *) wire        ila15v_tx_channel_seen   = pkt_channel_seen;
(* keep = "true" *) wire        ila15v_tx_lane_seen      = pkt_lane_seen;
(* keep = "true" *) wire        ila15v_tx_hard_err       = hard_err;
(* keep = "true" *) wire        ila15v_tx_hard_err_seen  = pkt_hard_err_seen;

endmodule
