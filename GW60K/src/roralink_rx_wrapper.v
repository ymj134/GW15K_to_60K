// ============================================================================
// 60K RoraLink RX-only wrapper
// ----------------------------------------------------------------------------
// Wraps SerDes_Top / RoraLink 8B10B RX-only IP and exposes a compact RX user
// interface plus link-status signals.
// ============================================================================
`timescale 1ns / 1ps

module roralink_rx_wrapper (
    input  wire        init_clk,
    input  wire        global_rst,

    output wire        link_reset,
    output wire        sys_reset,
    output wire [31:0] rx_data,
    output wire [3:0]  rx_strb,
    output wire        rx_valid,
    output wire        rx_last,
    output wire        crc_pass_fail_n,
    output wire        crc_valid,
    output wire        hard_err,
    output wire        soft_err,
    output wire        frame_err,
    output wire        channel_up,
    output wire        lane_up,
    output wire        rx_clk,
    output wire        gt_pll_ok,
    output wire        rx_align_link,
    output wire        rx_pma_lock,
    output wire        rx_k_lock
);

SerDes_Top u_serdes_top(
    .RoraLink_8B10B_Top_link_reset_o         (link_reset),
    .RoraLink_8B10B_Top_sys_reset_o          (sys_reset),
    .RoraLink_8B10B_Top_user_rx_data_o       (rx_data),
    .RoraLink_8B10B_Top_user_rx_strb_o       (rx_strb),
    .RoraLink_8B10B_Top_user_rx_valid_o      (rx_valid),
    .RoraLink_8B10B_Top_user_rx_last_o       (rx_last),
    .RoraLink_8B10B_Top_crc_pass_fail_n_o    (crc_pass_fail_n),
    .RoraLink_8B10B_Top_crc_valid_o          (crc_valid),
    .RoraLink_8B10B_Top_hard_err_o           (hard_err),
    .RoraLink_8B10B_Top_soft_err_o           (soft_err),
    .RoraLink_8B10B_Top_frame_err_o          (frame_err),
    .RoraLink_8B10B_Top_channel_up_o         (channel_up),
    .RoraLink_8B10B_Top_lane_up_o            (lane_up),
    .RoraLink_8B10B_Top_gt_pcs_rx_clk_o      (rx_clk),
    .RoraLink_8B10B_Top_gt_pll_lock_o        (gt_pll_ok),
    .RoraLink_8B10B_Top_gt_rx_align_link_o   (rx_align_link),
    .RoraLink_8B10B_Top_gt_rx_pma_lock_o     (rx_pma_lock),
    .RoraLink_8B10B_Top_gt_rx_k_lock_o       (rx_k_lock),

    .RoraLink_8B10B_Top_user_clk_i           (rx_clk),
    .RoraLink_8B10B_Top_init_clk_i           (init_clk),
    .RoraLink_8B10B_Top_reset_i              (global_rst),
    .RoraLink_8B10B_Top_user_pll_locked_i    (gt_pll_ok),
    .RoraLink_8B10B_Top_gt_reset_i           (global_rst),
    .RoraLink_8B10B_Top_gt_pcs_rx_reset_i    (global_rst)
);

endmodule
