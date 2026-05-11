// ============================================================================
// 15K video segment packetizer for RoraLink Framing TX
// ----------------------------------------------------------------------------
// Generates segmented RGB888_32 colorbar packets.
// One RoraLink frame = one video segment packet.
//
// Packet format:
//   word0 : 32'hA55A_6002
//   word1 : {frame_id[15:0], line_id[10:0], segment_id[4:0]}
//   word2 : {format[7:0], x_base[11:0], payload_words[11:0]}
//   word3 : {flags[7:0], width[11:0], height[11:0]}
//   word4..last : {8'h00, RGB888}
// ============================================================================

`timescale 1ns / 1ps

module video_segment_packetizer #(
    parameter [31:0] MAGIC              = 32'hA55A_6002,
    parameter [7:0]  FORMAT_RGB888_32   = 8'h01,
    parameter [11:0] H_RES              = 12'd1280,
    parameter [11:0] V_RES              = 12'd720,
    parameter [10:0] LAST_LINE          = 11'd719,
    parameter [4:0]  LAST_SEG           = 5'd4,
    parameter [11:0] SEG_PIXELS_FULL    = 12'd256,
    parameter [11:0] SEG_PIXELS_LAST    = 12'd256,
    parameter [11:0] HEADER_WORDS       = 12'd4,
    parameter [15:0] TX_START_DELAY_MAX = 16'hFFFF,
    parameter [11:0] SEG_GAP_CYCLES     = 12'd4,
    // 720p60 line pacing:
    // 6.25G user clk ~= 156.25MHz: 156.25MHz/(720*60) ~= 3617 cycles/line.
    // 3.125G user clk ~= 78.125MHz: 78.125MHz/(720*60) ~= 1808 cycles/line.
    parameter        LINK_RATE_IS_3G125 = 1'b1,
    parameter [11:0] LINE_GAP_3G125     = 12'd488,
    parameter [11:0] LINE_GAP_6G25      = 12'd2296,
    parameter [11:0] COLOR_BAR_WIDTH    = 12'd160
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        channel_up,
    input  wire        lane_up,
    input  wire        hard_err,
    input  wire        tx_ready,

    output wire [31:0] tx_data,
    output wire [3:0]  tx_strb,
    output wire        tx_valid,
    output wire        tx_last,

    output wire        tx_fire_dbg,
    output wire [1:0]  tx_state_dbg,
    output wire [15:0] frame_id_dbg,
    output wire [10:0] line_id_dbg,
    output wire [4:0]  segment_id_dbg,
    output wire [11:0] word_idx_dbg,
    output wire [11:0] payload_words_dbg,
    output wire [11:0] x_base_dbg,
    output wire [11:0] pixel_x_dbg,
    output wire [31:0] packet_cnt_dbg,
    output wire [31:0] word_cnt_dbg,
    output wire [11:0] gap_cnt_dbg,
    output wire [11:0] gap_target_dbg,
    output wire [15:0] start_delay_dbg,
    output wire        start_done_dbg,
    output wire        fire_seen_dbg,
    output wire        last_seen_dbg,
    output wire        ready_seen_dbg,
    output wire        channel_seen_dbg,
    output wire        lane_seen_dbg,
    output wire        hard_err_seen_dbg
);

localparam [1:0] ST_WAIT  = 2'd0;
localparam [1:0] ST_SEND  = 2'd1;
localparam [1:0] ST_GAP   = 2'd2;

localparam [11:0] LINE_GAP_CYCLES = LINK_RATE_IS_3G125 ? LINE_GAP_3G125 : LINE_GAP_6G25;

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

wire tx_fire = tx_valid & tx_ready;

wire [11:0] payload_words  = (segment_id == LAST_SEG) ? SEG_PIXELS_LAST : SEG_PIXELS_FULL;
wire [11:0] x_base         = {segment_id[3:0], 8'h00}; // segment_id * 256, valid for 0..4
wire [11:0] packet_last_idx= HEADER_WORDS + payload_words - 12'd1;
wire [11:0] payload_idx    = word_idx - HEADER_WORDS;
wire [11:0] pixel_x        = x_base + payload_idx;
wire [23:0] payload_rgb;

wire [7:0] flags = {
    4'b0000,
    (segment_id == 5'd0),                              // bit3 SOL
    (segment_id == LAST_SEG),                          // bit2 EOL
    ((line_id == LAST_LINE) && (segment_id == LAST_SEG)), // bit1 EOF
    ((line_id == 11'd0) && (segment_id == 5'd0))        // bit0 SOF
};

video_colorbar_rgb #(
    .BAR_WIDTH(COLOR_BAR_WIDTH)
) u_colorbar_rgb (
    .x   (pixel_x),
    .y   (line_id),
    .rgb (payload_rgb)
);

function [31:0] make_tx_word;
    input [11:0] idx;
    begin
        case (idx)
            12'd0:   make_tx_word = MAGIC;
            12'd1:   make_tx_word = {frame_id, line_id, segment_id};
            12'd2:   make_tx_word = {FORMAT_RGB888_32, x_base, payload_words};
            12'd3:   make_tx_word = {flags, H_RES, V_RES};
            default: make_tx_word = {8'h00, payload_rgb};
        endcase
    end
endfunction

assign tx_data  = make_tx_word(word_idx);
assign tx_strb  = 4'hF;
assign tx_valid = (tx_state == ST_SEND);
assign tx_last  = (tx_state == ST_SEND) && (word_idx == packet_last_idx);

always @(posedge clk or posedge rst) begin
    if (rst) begin
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
        if (tx_ready)   ready_seen <= 1'b1;
        if (channel_up) channel_seen <= 1'b1;
        if (lane_up)    lane_seen <= 1'b1;
        if (hard_err)   hard_err_seen <= 1'b1;

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

                        if (tx_last) begin
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

assign tx_fire_dbg       = tx_fire;
assign tx_state_dbg      = tx_state;
assign frame_id_dbg      = frame_id;
assign line_id_dbg       = line_id;
assign segment_id_dbg    = segment_id;
assign word_idx_dbg      = word_idx;
assign payload_words_dbg = payload_words;
assign x_base_dbg        = x_base;
assign pixel_x_dbg       = pixel_x;
assign packet_cnt_dbg    = packet_cnt;
assign word_cnt_dbg      = word_cnt;
assign gap_cnt_dbg       = gap_cnt;
assign gap_target_dbg    = gap_target;
assign start_delay_dbg   = tx_start_delay;
assign start_done_dbg    = (tx_state != ST_WAIT);
assign fire_seen_dbg     = tx_fire_seen;
assign last_seen_dbg     = tx_last_seen;
assign ready_seen_dbg    = ready_seen;
assign channel_seen_dbg  = channel_seen;
assign lane_seen_dbg     = lane_seen;
assign hard_err_seen_dbg = hard_err_seen;

endmodule
