// ============================================================================
// 15K RoraLink 8B10B TX-only Short Frame Pattern Test
// ----------------------------------------------------------------------------
// Purpose:
//   Minimal 15K TX source for 15K -> 60K RoraLink link bring-up.
//   Sends continuous 16-word frames after channel_up and user_tx_ready.
//
// Expected frame payload, repeated every 16 words:
//   word 0 : 32'h1234_5678
//   word 1 : 32'h9ABC_DEF0
//   word 2 : 32'h55AA_C33C
//   word 3 : 32'h0F1E_2D3C
//   ... repeated 4 times ...
//   user_tx_last is asserted on word 15.
//
// RoraLink IP configuration assumed:
//   TX-only Simplex / Framing / FlowControl None / BackChannel Timer
//   CRC enabled / Scrambler enabled / Little Endian enabled
//   1 lane / 4 Bytes per lane / 6.25Gbps / Q0 Lane0 / 125MHz refclk
// ============================================================================

`timescale 1ns / 1ps

module top (
    input  wire clk,      // 50MHz board clock, used as RoraLink init clock
    input  wire rst_n,
    output wire led
);

// --------------------------------------------------------------------------
// Version marker for ILA
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15s_tx_top_version = 32'h1506_2501;

// --------------------------------------------------------------------------
// RoraLink wires
// --------------------------------------------------------------------------
wire        rl_sys_reset;
wire        user_tx_ready;
wire        hard_err;
wire        channel_up;
wire        lane_up;
wire        tx_clk;
wire        gt_pll_ok;

wire [31:0] user_tx_data;
wire [3:0]  user_tx_strb;
wire        user_tx_valid;
wire        user_tx_last;

wire        global_rst = ~rst_n;
wire        tx_rst_async = global_rst | rl_sys_reset | ~gt_pll_ok;

// User logic reset synchronized to tx_clk.
reg [3:0] tx_rst_shr = 4'hF;
always @(posedge tx_clk or posedge tx_rst_async) begin
    if (tx_rst_async)
        tx_rst_shr <= 4'hF;
    else
        tx_rst_shr <= {tx_rst_shr[2:0], 1'b0};
end
wire tx_rst = tx_rst_shr[3];

// --------------------------------------------------------------------------
// RoraLink IP instance
// --------------------------------------------------------------------------
SerDes_Top u_serdes_top(
    .RoraLink_8B10B_Top_sys_reset_o          (rl_sys_reset),
    .RoraLink_8B10B_Top_user_tx_ready_o      (user_tx_ready),
    .RoraLink_8B10B_Top_hard_err_o           (hard_err),
    .RoraLink_8B10B_Top_channel_up_o         (channel_up),
    .RoraLink_8B10B_Top_lane_up_o            (lane_up),
    .RoraLink_8B10B_Top_gt_pcs_tx_clk_o      (tx_clk),
    .RoraLink_8B10B_Top_gt_pll_lock_o        (gt_pll_ok),

    .RoraLink_8B10B_Top_user_clk_i           (tx_clk),
    .RoraLink_8B10B_Top_init_clk_i           (clk),
    .RoraLink_8B10B_Top_reset_i              (global_rst),
    .RoraLink_8B10B_Top_user_pll_locked_i    (gt_pll_ok),

    .RoraLink_8B10B_Top_user_tx_data_i       (user_tx_data),
    .RoraLink_8B10B_Top_user_tx_strb_i       (user_tx_strb),
    .RoraLink_8B10B_Top_user_tx_valid_i      (user_tx_valid),
    .RoraLink_8B10B_Top_user_tx_last_i       (user_tx_last),

    .RoraLink_8B10B_Top_gt_reset_i           (global_rst),
    .RoraLink_8B10B_Top_gt_pcs_tx_reset_i    (global_rst)
);

// --------------------------------------------------------------------------
// 16-word short-frame generator
// --------------------------------------------------------------------------
localparam [15:0] TX_START_DELAY_MAX = 16'hFFFF;
localparam [3:0]  FRAME_WORDS_M1     = 4'd15;

reg [15:0] tx_start_delay = 16'd0;
reg        tx_enable      = 1'b0;
reg [3:0]  tx_word_idx    = 4'd0;
reg [31:0] tx_frame_cnt   = 32'd0;
reg [31:0] tx_word_cnt    = 32'd0;
reg        tx_fire_seen   = 1'b0;
reg        tx_last_seen   = 1'b0;
reg        ready_seen     = 1'b0;
reg        channel_seen   = 1'b0;
reg        lane_seen      = 1'b0;
reg        hard_err_seen  = 1'b0;

wire tx_fire = user_tx_valid & user_tx_ready;

function [31:0] pattern_word;
    input [1:0] sel;
    begin
        case (sel)
            2'd0: pattern_word = 32'h1234_5678;
            2'd1: pattern_word = 32'h9ABC_DEF0;
            2'd2: pattern_word = 32'h55AA_C33C;
            default: pattern_word = 32'h0F1E_2D3C;
        endcase
    end
endfunction

assign user_tx_data  = pattern_word(tx_word_idx[1:0]);
assign user_tx_strb  = 4'hF;
assign user_tx_valid = tx_enable;
assign user_tx_last  = tx_enable && (tx_word_idx == FRAME_WORDS_M1);

always @(posedge tx_clk or posedge tx_rst) begin
    if (tx_rst) begin
        tx_start_delay <= 16'd0;
        tx_enable      <= 1'b0;
        tx_word_idx    <= 4'd0;
        tx_frame_cnt   <= 32'd0;
        tx_word_cnt    <= 32'd0;
        tx_fire_seen   <= 1'b0;
        tx_last_seen   <= 1'b0;
        ready_seen     <= 1'b0;
        channel_seen   <= 1'b0;
        lane_seen      <= 1'b0;
        hard_err_seen  <= 1'b0;
    end else begin
        if (user_tx_ready) ready_seen <= 1'b1;
        if (channel_up)    channel_seen <= 1'b1;
        if (lane_up)       lane_seen <= 1'b1;
        if (hard_err)      hard_err_seen <= 1'b1;

        if (!channel_up || hard_err) begin
            tx_start_delay <= 16'd0;
            tx_enable      <= 1'b0;
            tx_word_idx    <= 4'd0;
        end else if (!tx_enable) begin
            tx_word_idx <= 4'd0;
            if (tx_start_delay == TX_START_DELAY_MAX)
                tx_enable <= 1'b1;
            else
                tx_start_delay <= tx_start_delay + 1'b1;
        end else if (tx_fire) begin
            tx_fire_seen <= 1'b1;
            tx_word_cnt  <= tx_word_cnt + 1'b1;

            if (user_tx_last) begin
                tx_last_seen <= 1'b1;
                tx_frame_cnt <= tx_frame_cnt + 1'b1;
                tx_word_idx  <= 4'd0;
            end else begin
                tx_word_idx <= tx_word_idx + 1'b1;
            end
        end
    end
end

// --------------------------------------------------------------------------
// LED
// --------------------------------------------------------------------------
reg [25:0] led_cnt = 26'd0;
always @(posedge tx_clk or posedge tx_rst) begin
    if (tx_rst)
        led_cnt <= 26'd0;
    else
        led_cnt <= led_cnt + 1'b1;
end

// ON: link up and frames sent. Fast blink: hard error. Slow blink: waiting.
assign led = hard_err_seen ? led_cnt[22] :
             (channel_up && tx_last_seen) ? 1'b1 : led_cnt[25];

// --------------------------------------------------------------------------
// ILA signals, search prefix: ila15s_tx_
// --------------------------------------------------------------------------
(* keep = "true" *) wire        ila15s_tx_gt_pll_ok      = gt_pll_ok;
(* keep = "true" *) wire        ila15s_tx_rl_sys_reset   = rl_sys_reset;
(* keep = "true" *) wire        ila15s_tx_tx_rst         = tx_rst;
(* keep = "true" *) wire        ila15s_tx_lane_up        = lane_up;
(* keep = "true" *) wire        ila15s_tx_channel_up     = channel_up;
(* keep = "true" *) wire        ila15s_tx_user_tx_ready  = user_tx_ready;
(* keep = "true" *) wire        ila15s_tx_user_tx_valid  = user_tx_valid;
(* keep = "true" *) wire        ila15s_tx_user_tx_last   = user_tx_last;
(* keep = "true" *) wire [31:0] ila15s_tx_user_tx_data   = user_tx_data;
(* keep = "true" *) wire [3:0]  ila15s_tx_user_tx_strb   = user_tx_strb;
(* keep = "true" *) wire        ila15s_tx_fire           = tx_fire;
(* keep = "true" *) wire [3:0]  ila15s_tx_word_idx       = tx_word_idx;
(* keep = "true" *) wire [31:0] ila15s_tx_frame_cnt      = tx_frame_cnt;
(* keep = "true" *) wire [31:0] ila15s_tx_word_cnt       = tx_word_cnt;
(* keep = "true" *) wire [15:0] ila15s_tx_start_delay    = tx_start_delay;
(* keep = "true" *) wire        ila15s_tx_start_done     = tx_enable;
(* keep = "true" *) wire        ila15s_tx_fire_seen      = tx_fire_seen;
(* keep = "true" *) wire        ila15s_tx_last_seen      = tx_last_seen;
(* keep = "true" *) wire        ila15s_tx_ready_seen     = ready_seen;
(* keep = "true" *) wire        ila15s_tx_channel_seen   = channel_seen;
(* keep = "true" *) wire        ila15s_tx_lane_seen      = lane_seen;
(* keep = "true" *) wire        ila15s_tx_hard_err       = hard_err;
(* keep = "true" *) wire        ila15s_tx_hard_err_seen  = hard_err_seen;

endmodule
