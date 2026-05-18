// ============================================================================
// Camera RGB stream segment packetizer for RoraLink Framing TX
// ----------------------------------------------------------------------------
// Input FIFO data width is 64-bit:
//   fifo_rd_data = {pixel1_rgb32, pixel0_rgb32}
//   pixel*_rgb32 = {8'h00, R, G, B}
//
// Output packet format is kept identical to the 60K B2 receiver:
//   word0 : 32'hA55A_6002
//   word1 : {frame_id[15:0], line_id[10:0], segment_id[4:0]}
//   word2 : {format[7:0], x_base[11:0], payload_words[11:0]}
//   word3 : {flags[7:0], width[11:0], height[11:0]}
//   word4..last : RGB888_32 payload, one pixel per 32-bit word
//
// This packetizer is data-driven. Before starting a segment, it waits until
// the async FIFO contains enough 64-bit pairs for the whole segment. This avoids
// sending invalid pixels if the camera stream is slower than the RoraLink TX.
// ============================================================================

`timescale 1ns / 1ps

module camera_stream_segment_packetizer #(
    parameter [31:0] MAGIC             = 32'hA55A_6002,
    parameter [7:0]  FORMAT_RGB888_32  = 8'h01,
    parameter [11:0] H_RES             = 12'd1280,
    parameter [11:0] V_RES             = 12'd720,
    parameter [10:0] LAST_LINE         = 11'd719,
    parameter [4:0]  LAST_SEG          = 5'd4,
    parameter [11:0] SEG_PIXELS_FULL   = 12'd256,
    parameter [11:0] SEG_PIXELS_LAST   = 12'd256,
    parameter [11:0] HEADER_WORDS      = 12'd4,
    parameter [15:0] START_PREFILL_PAIRS = 16'd512
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        channel_up,
    input  wire        lane_up,
    input  wire        hard_err,
    input  wire        tx_ready,

    input  wire [63:0] fifo_rd_data,
    input  wire        fifo_empty,
    input  wire [15:0] fifo_rd_count,
    output wire        fifo_rd_en,

    output wire [31:0] tx_data,
    output wire [3:0]  tx_strb,
    output wire        tx_valid,
    output wire        tx_last,

    output wire        tx_fire_dbg,
    output reg  [2:0]  tx_state_dbg,
    output reg  [15:0] frame_id_dbg,
    output reg  [10:0] line_id_dbg,
    output reg  [4:0]  segment_id_dbg,
    output reg  [11:0] word_idx_dbg,
    output wire [11:0] payload_words_dbg,
    output wire [11:0] x_base_dbg,
    output reg  [31:0] packet_cnt_dbg,
    output reg  [31:0] word_cnt_dbg,
    output reg         start_done_dbg,
    output reg         fire_seen_dbg,
    output reg         last_seen_dbg,
    output reg         ready_seen_dbg,
    output reg         channel_seen_dbg,
    output reg         lane_seen_dbg,
    output reg         hard_err_seen_dbg,
    output reg         fifo_empty_seen_dbg,
    output reg         fifo_underflow_seen_dbg
);

localparam [2:0] ST_WAIT      = 3'd0;
localparam [2:0] ST_SEG_WAIT  = 3'd1;
localparam [2:0] ST_HEADER    = 3'd2;
localparam [2:0] ST_PAYLOAD   = 3'd3;
localparam [2:0] ST_REQ_PAIR  = 3'd4;
localparam [2:0] ST_LOAD_PAIR = 3'd5;

reg [2:0] tx_state = ST_WAIT;
reg [15:0] frame_id = 16'd0;
reg [10:0] line_id = 11'd0;
reg [4:0]  segment_id = 5'd0;
reg [11:0] header_idx = 12'd0;
reg [11:0] payload_idx = 12'd0;
reg [31:0] packet_cnt = 32'd0;
reg [31:0] word_cnt = 32'd0;

reg [63:0] pair_buf = 64'd0;
reg        pair_valid = 1'b0;
reg        pair_sel = 1'b0; // 0: low pixel, 1: high pixel

wire [11:0] payload_words = (segment_id == LAST_SEG) ? SEG_PIXELS_LAST : SEG_PIXELS_FULL;
wire [11:0] required_pairs = payload_words[11:1];
wire [11:0] x_base = {segment_id[3:0], 8'h00}; // segment_id * 256
wire [7:0]  flags = {
    4'b0000,
    (segment_id == 5'd0),
    (segment_id == LAST_SEG),
    ((line_id == LAST_LINE) && (segment_id == LAST_SEG)),
    ((line_id == 11'd0) && (segment_id == 5'd0))
};

wire [31:0] header_word =
    (header_idx == 12'd0) ? MAGIC :
    (header_idx == 12'd1) ? {frame_id, line_id, segment_id} :
    (header_idx == 12'd2) ? {FORMAT_RGB888_32, x_base, payload_words} :
                             {flags, H_RES, V_RES};

wire [31:0] payload_word = pair_sel ? pair_buf[63:32] : pair_buf[31:0];

assign tx_valid = (tx_state == ST_HEADER) || ((tx_state == ST_PAYLOAD) && pair_valid);
assign tx_data  = (tx_state == ST_HEADER) ? header_word : payload_word;
assign tx_strb  = 4'hF;
assign tx_last  = (tx_state == ST_PAYLOAD) && pair_valid && (payload_idx == (payload_words - 12'd1));
assign tx_fire_dbg = tx_valid & tx_ready;
assign fifo_rd_en = (tx_state == ST_REQ_PAIR) && !fifo_empty;

wire can_start_segment = (fifo_rd_count >= {4'd0, required_pairs});
wire can_start_stream  = (fifo_rd_count >= START_PREFILL_PAIRS);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        tx_state                <= ST_WAIT;
        frame_id                <= 16'd0;
        line_id                 <= 11'd0;
        segment_id              <= 5'd0;
        header_idx              <= 12'd0;
        payload_idx             <= 12'd0;
        packet_cnt              <= 32'd0;
        word_cnt                <= 32'd0;
        pair_buf                <= 64'd0;
        pair_valid              <= 1'b0;
        pair_sel                <= 1'b0;
        start_done_dbg          <= 1'b0;
        fire_seen_dbg           <= 1'b0;
        last_seen_dbg           <= 1'b0;
        ready_seen_dbg          <= 1'b0;
        channel_seen_dbg        <= 1'b0;
        lane_seen_dbg           <= 1'b0;
        hard_err_seen_dbg       <= 1'b0;
        fifo_empty_seen_dbg     <= 1'b0;
        fifo_underflow_seen_dbg <= 1'b0;
    end else begin
        if (tx_ready)   ready_seen_dbg   <= 1'b1;
        if (channel_up) channel_seen_dbg <= 1'b1;
        if (lane_up)    lane_seen_dbg    <= 1'b1;
        if (hard_err)   hard_err_seen_dbg <= 1'b1;
        if (fifo_empty) fifo_empty_seen_dbg <= 1'b1;

        if (!channel_up || hard_err) begin
            tx_state    <= ST_WAIT;
            frame_id    <= 16'd0;
            line_id     <= 11'd0;
            segment_id  <= 5'd0;
            header_idx  <= 12'd0;
            payload_idx <= 12'd0;
            pair_valid  <= 1'b0;
            pair_sel    <= 1'b0;
            start_done_dbg <= 1'b0;
        end else begin
            case (tx_state)
                ST_WAIT: begin
                    header_idx  <= 12'd0;
                    payload_idx <= 12'd0;
                    pair_valid  <= 1'b0;
                    pair_sel    <= 1'b0;
                    if (tx_ready && can_start_stream) begin
                        tx_state       <= ST_SEG_WAIT;
                        start_done_dbg <= 1'b1;
                    end
                end

                ST_SEG_WAIT: begin
                    header_idx  <= 12'd0;
                    payload_idx <= 12'd0;
                    pair_valid  <= 1'b0;
                    pair_sel    <= 1'b0;
                    if (tx_ready && can_start_segment)
                        tx_state <= ST_HEADER;
                end

                ST_HEADER: begin
                    if (tx_fire_dbg) begin
                        fire_seen_dbg <= 1'b1;
                        word_cnt <= word_cnt + 1'b1;
                        if (header_idx == HEADER_WORDS - 12'd1) begin
                            header_idx  <= 12'd0;
                            payload_idx <= 12'd0;
                            tx_state    <= ST_PAYLOAD;
                        end else begin
                            header_idx <= header_idx + 1'b1;
                        end
                    end
                end

                ST_PAYLOAD: begin
                    if (!pair_valid) begin
                        if (!fifo_empty)
                            tx_state <= ST_REQ_PAIR;
                        else
                            fifo_underflow_seen_dbg <= 1'b1;
                    end else if (tx_fire_dbg) begin
                        fire_seen_dbg <= 1'b1;
                        word_cnt <= word_cnt + 1'b1;

                        if (tx_last) begin
                            last_seen_dbg <= 1'b1;
                            packet_cnt <= packet_cnt + 1'b1;
                            pair_valid <= 1'b0;
                            pair_sel <= 1'b0;

                            if (segment_id == LAST_SEG) begin
                                segment_id <= 5'd0;
                                if (line_id == LAST_LINE) begin
                                    line_id <= 11'd0;
                                    frame_id <= frame_id + 1'b1;
                                end else begin
                                    line_id <= line_id + 1'b1;
                                end
                            end else begin
                                segment_id <= segment_id + 1'b1;
                            end
                            tx_state <= ST_SEG_WAIT;
                        end else begin
                            payload_idx <= payload_idx + 1'b1;
                            if (!pair_sel) begin
                                pair_sel <= 1'b1;
                            end else begin
                                pair_sel <= 1'b0;
                                pair_valid <= 1'b0;
                            end
                        end
                    end
                end

                ST_REQ_PAIR: begin
                    // fifo_rd_en is asserted combinationally in this state.
                    // FIFO rd_data is registered, so wait one more cycle.
                    tx_state <= ST_LOAD_PAIR;
                end

                ST_LOAD_PAIR: begin
                    pair_buf   <= fifo_rd_data;
                    pair_valid <= 1'b1;
                    pair_sel   <= 1'b0;
                    tx_state   <= ST_PAYLOAD;
                end

                default: begin
                    tx_state <= ST_WAIT;
                end
            endcase
        end
    end
end

always @(*) begin
    tx_state_dbg   = tx_state;
    frame_id_dbg   = frame_id;
    line_id_dbg    = line_id;
    segment_id_dbg = segment_id;
    word_idx_dbg   = (tx_state == ST_HEADER) ? header_idx : (HEADER_WORDS + payload_idx);
    packet_cnt_dbg = packet_cnt;
    word_cnt_dbg   = word_cnt;
end

assign payload_words_dbg = payload_words;
assign x_base_dbg = x_base;

endmodule
