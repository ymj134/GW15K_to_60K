// ============================================================================
// 15K RoraLink TX-only wrapper
// ----------------------------------------------------------------------------
// Wraps the generated SerDes_Top / RoraLink 8B10B TX-only IP and provides
// a clean TX user-clock/reset/status interface to the packetizer.
// ============================================================================

`timescale 1ns / 1ps

module roralink_tx_wrapper (
    input  wire        init_clk,      // 50MHz board clock, RoraLink init clock
    input  wire        rst_n,

    output wire        tx_clk,
    output wire        tx_rst,
    output wire        rl_sys_reset,
    output wire        gt_pll_ok,
    output wire        lane_up,
    output wire        channel_up,
    output wire        hard_err,
    output wire        user_tx_ready,

    input  wire [31:0] user_tx_data,
    input  wire [3:0]  user_tx_strb,
    input  wire        user_tx_valid,
    input  wire        user_tx_last
);

wire global_rst = ~rst_n;
wire tx_rst_async = global_rst | rl_sys_reset | ~gt_pll_ok;

// User logic reset synchronized to tx_clk.
reg [3:0] tx_rst_shr = 4'hF;
always @(posedge tx_clk or posedge tx_rst_async) begin
    if (tx_rst_async)
        tx_rst_shr <= 4'hF;
    else
        tx_rst_shr <= {tx_rst_shr[2:0], 1'b0};
end
assign tx_rst = tx_rst_shr[3];

SerDes_Top u_serdes_top(
    .RoraLink_8B10B_Top_sys_reset_o          (rl_sys_reset),
    .RoraLink_8B10B_Top_user_tx_ready_o      (user_tx_ready),
    .RoraLink_8B10B_Top_hard_err_o           (hard_err),
    .RoraLink_8B10B_Top_channel_up_o         (channel_up),
    .RoraLink_8B10B_Top_lane_up_o            (lane_up),
    .RoraLink_8B10B_Top_gt_pcs_tx_clk_o      (tx_clk),
    .RoraLink_8B10B_Top_gt_pll_lock_o        (gt_pll_ok),

    .RoraLink_8B10B_Top_user_clk_i           (tx_clk),
    .RoraLink_8B10B_Top_init_clk_i           (init_clk),
    .RoraLink_8B10B_Top_reset_i              (global_rst),
    .RoraLink_8B10B_Top_user_pll_locked_i    (gt_pll_ok),

    .RoraLink_8B10B_Top_user_tx_data_i       (user_tx_data),
    .RoraLink_8B10B_Top_user_tx_strb_i       (user_tx_strb),
    .RoraLink_8B10B_Top_user_tx_valid_i      (user_tx_valid),
    .RoraLink_8B10B_Top_user_tx_last_i       (user_tx_last),

    .RoraLink_8B10B_Top_gt_reset_i           (global_rst),
    .RoraLink_8B10B_Top_gt_pcs_tx_reset_i    (global_rst)
);

endmodule
