// ============================================================================
// 15K RoraLink 8B10B TX-only Segmented 720p Colorbar Source - B1
// ----------------------------------------------------------------------------
// Purpose:
//   Generate segmented 720p RGB888_32 colorbar packets and transmit them over
//   RoraLink Framing mode. This is B1: link + segmented packet verification,
//   before connecting the 60K DDR writer.
//
// RoraLink IP configuration assumed:
//   TX-only Simplex / Framing / FlowControl None / BackChannel Timer
//   Scrambler enabled / CRC enabled / Little Endian enabled
//   1 lane / 4 Bytes per lane / 6.25Gbps or 3.125Gbps / Q0 Lane0 / 125MHz refclk
//
// Packet format, one RoraLink frame per segment:
//   word0 : 32'hA55A_6002
//   word1 : {frame_id[15:0], line_id[10:0], segment_id[4:0]}
//   word2 : {format[7:0], x_base[11:0], payload_words[11:0]}
//           format = 8'h01 = RGB888 packed as {8'h00, R, G, B}
//   word3 : {flags[7:0], width[11:0], height[11:0]}
//           flags[0] = SOF segment: line0/seg0
//           flags[1] = EOF segment: line719/seg4
//           flags[2] = EOL segment: seg4
//           flags[3] = SOL segment: seg0
//   word4..last : pixel payload, one 32-bit word per pixel
//
// Segmentation:
//   1280 pixels/line -> 5 segments/line
//   seg0..seg4: 256 pixels each
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
(* keep = "true" *) wire [31:0] ila15v_tx_top_version = 32'h1507_2001;

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
// Video segmented packet generator
// --------------------------------------------------------------------------
localparam [31:0] MAGIC              = 32'hA55A_6002;
localparam [7:0]  FORMAT_RGB888_32   = 8'h01;
localparam [11:0] H_RES              = 12'd1280;
localparam [11:0] V_RES              = 12'd720;
localparam [10:0] LAST_LINE          = 11'd719;
localparam [4:0]  LAST_SEG           = 5'd4;
localparam [11:0] SEG_PIXELS_FULL    = 12'd256;
localparam [11:0] SEG_PIXELS_LAST    = 12'd256;
localparam [11:0] HEADER_WORDS       = 12'd4;
localparam [15:0] TX_START_DELAY_MAX = 16'hFFFF;

// Small gap between RoraLink frames makes framing/CRC observation cleaner.
localparam [11:0] SEG_GAP_CYCLES     = 12'd4;

// Approximate 720p60 line pacing.
// Per line words = 5*4 headers + 1280 payload = 1300.
// 6.25G  RoraLink user clock ~= 156.25MHz: 156.25MHz / (720*60) ~= 3617 cycles/line.
// 3.125G RoraLink user clock ~=  78.125MHz: 78.125MHz / (720*60) ~= 1808 cycles/line.
// Default below is for 6.25G. Set LINK_RATE_IS_3G125 to 1'b1 only when both
// 15K/60K RoraLink IPs are regenerated as 3.125G.
localparam        LINK_RATE_IS_3G125 = 1'b0;
localparam [11:0] LINE_GAP_CYCLES    = LINK_RATE_IS_3G125 ? 12'd488 : 12'd2296;

localparam [1:0] ST_WAIT  = 2'd0;
localparam [1:0] ST_SEND  = 2'd1;
localparam [1:0] ST_GAP   = 2'd2;

reg [1:0]  tx_state       = ST_WAIT;
reg [15:0] tx_start_delay = 16'd0;
reg [11:0] gap_cnt        = 12'd0;
reg [11:0] gap_target     = 12'd0;

reg [15:0] frame_id       = 16'd0;
reg [10:0] line_id        = 11'd0;
reg [4:0]  segment_id     = 5'd0;
reg [11:0] word_idx       = 12'd0;
reg [31:0] packet_cnt     = 32'd0;
reg [31:0] word_cnt       = 32'd0;

reg        tx_fire_seen   = 1'b0;
reg        tx_last_seen   = 1'b0;
reg        ready_seen     = 1'b0;
reg        channel_seen   = 1'b0;
reg        lane_seen      = 1'b0;
reg        hard_err_seen  = 1'b0;

wire tx_fire = user_tx_valid & user_tx_ready;

wire [11:0] payload_words = (segment_id == LAST_SEG) ? SEG_PIXELS_LAST : SEG_PIXELS_FULL;
wire [11:0] x_base        = {segment_id[3:0], 8'h00}; // segment_id * 256, valid for 0..4
wire [11:0] packet_last_idx = HEADER_WORDS + payload_words - 12'd1;
wire [11:0] payload_idx   = word_idx - HEADER_WORDS;
wire [11:0] pixel_x       = x_base + payload_idx;

wire [7:0] flags = {
    4'b0000,
    (segment_id == 5'd0),                         // bit3 SOL
    (segment_id == LAST_SEG),                     // bit2 EOL
    ((line_id == LAST_LINE) && (segment_id == LAST_SEG)), // bit1 EOF
    ((line_id == 11'd0) && (segment_id == 5'd0))  // bit0 SOF
};

function [23:0] colorbar_rgb;
    input [11:0] x;
    input [10:0] y;
    begin
        // 8 vertical bars, 160 pixels each for 1280x720. y is reserved for future patterns.
        if (x < 12'd160)
            colorbar_rgb = 24'hFF_FF_FF; // white
        else if (x < 12'd320)
            colorbar_rgb = 24'hFF_FF_00; // yellow
        else if (x < 12'd480)
            colorbar_rgb = 24'h00_FF_FF; // cyan
        else if (x < 12'd640)
            colorbar_rgb = 24'h00_FF_00; // green
        else if (x < 12'd800)
            colorbar_rgb = 24'hFF_00_FF; // magenta
        else if (x < 12'd960)
            colorbar_rgb = 24'hFF_00_00; // red
        else if (x < 12'd1120)
            colorbar_rgb = 24'h00_00_FF; // blue
        else
            colorbar_rgb = 24'h00_00_00; // black
    end
endfunction

function [31:0] make_tx_word;
    input [11:0] idx;
    begin
        case (idx)
            12'd0: make_tx_word = MAGIC;
            12'd1: make_tx_word = {frame_id, line_id, segment_id};
            12'd2: make_tx_word = {FORMAT_RGB888_32, x_base, payload_words};
            12'd3: make_tx_word = {flags, H_RES, V_RES};
            default: make_tx_word = {8'h00, colorbar_rgb(pixel_x, line_id)};
        endcase
    end
endfunction

assign user_tx_data  = make_tx_word(word_idx);
assign user_tx_strb  = 4'hF;
assign user_tx_valid = (tx_state == ST_SEND);
assign user_tx_last  = (tx_state == ST_SEND) && (word_idx == packet_last_idx);

always @(posedge tx_clk or posedge tx_rst) begin
    if (tx_rst) begin
        tx_state       <= ST_WAIT;
        tx_start_delay <= 16'd0;
        gap_cnt        <= 12'd0;
        gap_target     <= 12'd0;
        frame_id       <= 16'd0;
        line_id        <= 11'd0;
        segment_id     <= 5'd0;
        word_idx       <= 12'd0;
        packet_cnt     <= 32'd0;
        word_cnt       <= 32'd0;
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
            tx_state       <= ST_WAIT;
            tx_start_delay <= 16'd0;
            gap_cnt        <= 12'd0;
            word_idx       <= 12'd0;
        end else begin
            case (tx_state)
                ST_WAIT: begin
                    word_idx <= 12'd0;
                    gap_cnt  <= 12'd0;
                    if (tx_start_delay == TX_START_DELAY_MAX)
                        tx_state <= ST_SEND;
                    else
                        tx_start_delay <= tx_start_delay + 1'b1;
                end

                ST_SEND: begin
                    if (tx_fire) begin
                        tx_fire_seen <= 1'b1;
                        word_cnt     <= word_cnt + 1'b1;

                        if (user_tx_last) begin
                            tx_last_seen <= 1'b1;
                            packet_cnt   <= packet_cnt + 1'b1;
                            word_idx     <= 12'd0;

                            if (segment_id == LAST_SEG) begin
                                segment_id <= 5'd0;
                                if (line_id == LAST_LINE) begin
                                    line_id  <= 11'd0;
                                    frame_id <= frame_id + 1'b1;
                                end else begin
                                    line_id <= line_id + 1'b1;
                                end
                                gap_target <= LINE_GAP_CYCLES;
                            end else begin
                                segment_id <= segment_id + 1'b1;
                                gap_target <= SEG_GAP_CYCLES;
                            end
                            gap_cnt  <= 12'd0;
                            tx_state <= ST_GAP;
                        end else begin
                            word_idx <= word_idx + 1'b1;
                        end
                    end
                end

                ST_GAP: begin
                    if (gap_cnt >= gap_target) begin
                        tx_state <= ST_SEND;
                        gap_cnt  <= 12'd0;
                    end else begin
                        gap_cnt <= gap_cnt + 1'b1;
                    end
                end

                default: tx_state <= ST_WAIT;
            endcase
        end
    end
end

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

assign led = hard_err_seen ? led_cnt[22] :
             (channel_up && tx_last_seen) ? 1'b1 : led_cnt[25];

// --------------------------------------------------------------------------
// ILA signals, search prefix: ila15v_tx_
// --------------------------------------------------------------------------
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
(* keep = "true" *) wire        ila15v_tx_fire           = tx_fire;
(* keep = "true" *) wire [1:0]  ila15v_tx_state          = tx_state;
(* keep = "true" *) wire [15:0] ila15v_tx_frame_id       = frame_id;
(* keep = "true" *) wire [10:0] ila15v_tx_line_id        = line_id;
(* keep = "true" *) wire [4:0]  ila15v_tx_segment_id     = segment_id;
(* keep = "true" *) wire [11:0] ila15v_tx_word_idx       = word_idx;
(* keep = "true" *) wire [11:0] ila15v_tx_payload_words  = payload_words;
(* keep = "true" *) wire [11:0] ila15v_tx_x_base         = x_base;
(* keep = "true" *) wire [11:0] ila15v_tx_pixel_x        = pixel_x;
(* keep = "true" *) wire [31:0] ila15v_tx_packet_cnt     = packet_cnt;
(* keep = "true" *) wire [31:0] ila15v_tx_word_cnt       = word_cnt;
(* keep = "true" *) wire [11:0] ila15v_tx_gap_cnt        = gap_cnt;
(* keep = "true" *) wire [11:0] ila15v_tx_gap_target     = gap_target;
(* keep = "true" *) wire [15:0] ila15v_tx_start_delay    = tx_start_delay;
(* keep = "true" *) wire        ila15v_tx_start_done     = (tx_state != ST_WAIT);
(* keep = "true" *) wire        ila15v_tx_fire_seen      = tx_fire_seen;
(* keep = "true" *) wire        ila15v_tx_last_seen      = tx_last_seen;
(* keep = "true" *) wire        ila15v_tx_ready_seen     = ready_seen;
(* keep = "true" *) wire        ila15v_tx_channel_seen   = channel_seen;
(* keep = "true" *) wire        ila15v_tx_lane_seen      = lane_seen;
(* keep = "true" *) wire        ila15v_tx_hard_err       = hard_err;
(* keep = "true" *) wire        ila15v_tx_hard_err_seen  = hard_err_seen;

endmodule
