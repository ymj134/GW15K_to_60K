// ============================================================================
// Camera 720p cropped RGB-pair stream -> RoraLink segmented packet FIFO writer
// ----------------------------------------------------------------------------
// Clock domain: camera byte_clk / RGB pair clock.
//
// Input crop stream:
//   crop_pair_data = {pixel1_rgb32, pixel0_rgb32}
//   pixel*_rgb32   = {8'h00, R, G, B}
//
// Output packet FIFO word:
//   packet_fifo_wr_data = {last, data[31:0]}
//
// Packet format is identical to the existing 60K B2 receiver:
//   word0 : 32'hA55A_6002
//   word1 : {frame_id[15:0], line_id[10:0], segment_id[4:0]}
//   word2 : {format[7:0], x_base[11:0], payload_words[11:0]}
//   word3 : {flags[7:0], width[11:0], height[11:0]}
//   payload: 256 RGB888_32 words, last asserted on final payload word.
//
// Design intent:
//   1. Camera/crop domain owns frame/line/segment alignment.
//   2. TX domain no longer invents headers from a pixel-only FIFO.
//   3. Temporary 64-bit pair FIFO absorbs the 4 header-word overhead.
// ============================================================================

`timescale 1ns / 1ps

module camera_packet_builder_720p #(
    parameter [31:0] MAGIC                = 32'hA55A_6002,
    parameter [7:0]  FORMAT_RGB888_32     = 8'h01,
    parameter [11:0] H_RES                = 12'd1280,
    parameter [11:0] V_RES                = 12'd720,
    parameter [10:0] LAST_LINE            = 11'd719,
    parameter [4:0]  LAST_SEG             = 5'd4,
    parameter [11:0] SEG_PIXELS           = 12'd256,
    parameter integer PAIR_FIFO_ADDR_WIDTH = 12,
    parameter [15:0] START_PREFILL_PAIRS  = 16'd512
)(
    input  wire        clk,
    input  wire        rst,          // active-high, async-safe reset from top
    input  wire        link_active,  // synchronized to clk by top

    input  wire        crop_valid,
    input  wire        crop_sof,
    input  wire [63:0] crop_pair_data,

    input  wire        packet_fifo_full,
    input  wire        packet_fifo_almost_full,
    output reg         packet_fifo_wr_en,
    output reg  [32:0] packet_fifo_wr_data,

    output wire        pair_fifo_full_dbg,
    output wire        pair_fifo_empty_dbg,
    output wire [15:0] pair_fifo_wr_count_dbg,
    output wire [15:0] pair_fifo_rd_count_dbg,
    output wire        pair_fifo_rd_en_dbg,

    output reg  [3:0]  builder_state_dbg,
    output reg  [15:0] frame_id_dbg,
    output reg  [10:0] line_id_dbg,
    output reg  [4:0]  segment_id_dbg,
    output reg  [7:0]  pair_idx_dbg,
    output reg  [11:0] word_idx_dbg,
    output reg  [31:0] packet_cnt_dbg,
    output reg  [31:0] packet_word_cnt_dbg,
    output reg         stream_active_dbg,
    output reg         sof_seen_dbg,
    output reg         packet_wr_seen_dbg,
    output reg         packet_last_seen_dbg,
    output reg         pair_fifo_overflow_seen_dbg,
    output reg         pair_fifo_empty_seen_dbg,
    output reg         packet_fifo_full_seen_dbg,
    output reg         packet_fifo_afull_seen_dbg
);

localparam [3:0] ST_WAIT_LINK   = 4'd0;
localparam [3:0] ST_WAIT_PREFILL= 4'd1;
localparam [3:0] ST_HEADER      = 4'd2;
localparam [3:0] ST_REQ_PAIR    = 4'd3;
localparam [3:0] ST_LOAD_PAIR   = 4'd4;
localparam [3:0] ST_PAY0        = 4'd5;
localparam [3:0] ST_PAY1        = 4'd6;

localparam [11:0] HEADER_WORDS = 12'd4;
localparam [7:0]  PAIRS_PER_SEG = SEG_PIXELS[8:1]; // 256 pixels -> 128 pairs

wire pair_fifo_rst = rst | ~link_active;

reg stream_active;
wire start_stream = crop_valid && crop_sof && link_active && !pair_fifo_full;
wire pair_fifo_wr_en = crop_valid && (stream_active || start_stream) && !pair_fifo_full;

always @(posedge clk or posedge pair_fifo_rst) begin
    if (pair_fifo_rst) begin
        stream_active                 <= 1'b0;
        sof_seen_dbg                  <= 1'b0;
        pair_fifo_overflow_seen_dbg   <= 1'b0;
    end else begin
        if (start_stream) begin
            stream_active <= 1'b1;
            sof_seen_dbg  <= 1'b1;
        end

        if (crop_valid && (stream_active || start_stream) && pair_fifo_full) begin
            pair_fifo_overflow_seen_dbg <= 1'b1;
            // Stop taking this frame. The flag is latched; normal operation
            // should never reach this path with enough FIFO depth/link BW.
            stream_active <= 1'b0;
        end
    end
end

wire [63:0] pair_fifo_rd_data;
wire        pair_fifo_empty;
wire        pair_fifo_full;
wire [PAIR_FIFO_ADDR_WIDTH:0] pair_fifo_wr_count_raw;
wire [PAIR_FIFO_ADDR_WIDTH:0] pair_fifo_rd_count_raw;
reg         pair_fifo_rd_en;

async_fifo_gray #(
    .DATA_WIDTH (64),
    .ADDR_WIDTH (PAIR_FIFO_ADDR_WIDTH)
) u_pair_elastic_fifo (
    .rst      (pair_fifo_rst),
    .wr_clk   (clk),
    .wr_en    (pair_fifo_wr_en),
    .wr_data  (crop_pair_data),
    .full     (pair_fifo_full),
    .wr_count (pair_fifo_wr_count_raw),
    .rd_clk   (clk),
    .rd_en    (pair_fifo_rd_en),
    .rd_data  (pair_fifo_rd_data),
    .empty    (pair_fifo_empty),
    .rd_count (pair_fifo_rd_count_raw)
);

assign pair_fifo_full_dbg      = pair_fifo_full;
assign pair_fifo_empty_dbg     = pair_fifo_empty;
assign pair_fifo_wr_count_dbg  = {{(16-(PAIR_FIFO_ADDR_WIDTH+1)){1'b0}}, pair_fifo_wr_count_raw};
assign pair_fifo_rd_count_dbg  = {{(16-(PAIR_FIFO_ADDR_WIDTH+1)){1'b0}}, pair_fifo_rd_count_raw};
assign pair_fifo_rd_en_dbg     = pair_fifo_rd_en;

reg [3:0]  state;
reg [15:0] frame_id;
reg [10:0] line_id;
reg [4:0]  segment_id;
reg [1:0]  header_idx;
reg [7:0]  pair_idx;
reg [63:0] pair_buf;
reg [31:0] packet_cnt;
reg [31:0] packet_word_cnt;

wire [11:0] x_base = {segment_id, 8'd0}; // segment_id * 256
wire [31:0] header_word =
    (header_idx == 2'd0) ? MAGIC :
    (header_idx == 2'd1) ? {frame_id, line_id, segment_id} :
    (header_idx == 2'd2) ? {FORMAT_RGB888_32, x_base, SEG_PIXELS} :
                            {8'h00, H_RES, V_RES};

wire prefill_ok = (pair_fifo_rd_count_dbg >= START_PREFILL_PAIRS);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                       <= ST_WAIT_LINK;
        frame_id                    <= 16'd0;
        line_id                     <= 11'd0;
        segment_id                  <= 5'd0;
        header_idx                  <= 2'd0;
        pair_idx                    <= 8'd0;
        pair_buf                    <= 64'd0;
        pair_fifo_rd_en             <= 1'b0;
        packet_fifo_wr_en           <= 1'b0;
        packet_fifo_wr_data         <= 33'd0;
        packet_cnt                  <= 32'd0;
        packet_word_cnt             <= 32'd0;
        packet_wr_seen_dbg          <= 1'b0;
        packet_last_seen_dbg        <= 1'b0;
        pair_fifo_empty_seen_dbg    <= 1'b0;
        packet_fifo_full_seen_dbg   <= 1'b0;
        packet_fifo_afull_seen_dbg  <= 1'b0;
    end else begin
        pair_fifo_rd_en   <= 1'b0;
        packet_fifo_wr_en <= 1'b0;

        if (packet_fifo_full)
            packet_fifo_full_seen_dbg <= 1'b1;
        if (packet_fifo_almost_full)
            packet_fifo_afull_seen_dbg <= 1'b1;

        if (!link_active) begin
            state      <= ST_WAIT_LINK;
            frame_id   <= 16'd0;
            line_id    <= 11'd0;
            segment_id <= 5'd0;
            header_idx <= 2'd0;
            pair_idx   <= 8'd0;
        end else begin
            case (state)
                ST_WAIT_LINK: begin
                    header_idx <= 2'd0;
                    pair_idx   <= 8'd0;
                    if (stream_active)
                        state <= ST_WAIT_PREFILL;
                end

                ST_WAIT_PREFILL: begin
                    header_idx <= 2'd0;
                    pair_idx   <= 8'd0;
                    if (prefill_ok)
                        state <= ST_HEADER;
                end

                ST_HEADER: begin
                    if (!packet_fifo_full) begin
                        packet_fifo_wr_en   <= 1'b1;
                        packet_fifo_wr_data <= {1'b0, header_word};
                        packet_wr_seen_dbg  <= 1'b1;
                        packet_word_cnt     <= packet_word_cnt + 1'b1;

                        if (header_idx == 2'd3) begin
                            header_idx <= 2'd0;
                            pair_idx   <= 8'd0;
                            state      <= ST_REQ_PAIR;
                        end else begin
                            header_idx <= header_idx + 1'b1;
                        end
                    end
                end

                ST_REQ_PAIR: begin
                    if (!pair_fifo_empty) begin
                        pair_fifo_rd_en <= 1'b1;
                        state           <= ST_LOAD_PAIR;
                    end else begin
                        pair_fifo_empty_seen_dbg <= 1'b1;
                    end
                end

                ST_LOAD_PAIR: begin
                    pair_buf <= pair_fifo_rd_data;
                    state    <= ST_PAY0;
                end

                ST_PAY0: begin
                    if (!packet_fifo_full) begin
                        packet_fifo_wr_en   <= 1'b1;
                        packet_fifo_wr_data <= {1'b0, pair_buf[31:0]};
                        packet_wr_seen_dbg  <= 1'b1;
                        packet_word_cnt     <= packet_word_cnt + 1'b1;
                        state               <= ST_PAY1;
                    end
                end

                ST_PAY1: begin
                    if (!packet_fifo_full) begin
                        packet_fifo_wr_en   <= 1'b1;
                        packet_fifo_wr_data <= {(pair_idx == PAIRS_PER_SEG - 8'd1), pair_buf[63:32]};
                        packet_wr_seen_dbg  <= 1'b1;
                        packet_word_cnt     <= packet_word_cnt + 1'b1;

                        if (pair_idx == PAIRS_PER_SEG - 8'd1) begin
                            packet_last_seen_dbg <= 1'b1;
                            packet_cnt <= packet_cnt + 1'b1;
                            pair_idx   <= 8'd0;

                            if (segment_id == LAST_SEG) begin
                                segment_id <= 5'd0;
                                if (line_id == LAST_LINE) begin
                                    line_id  <= 11'd0;
                                    frame_id <= frame_id + 1'b1;
                                end else begin
                                    line_id <= line_id + 1'b1;
                                end
                            end else begin
                                segment_id <= segment_id + 1'b1;
                            end
                            state <= ST_HEADER;
                        end else begin
                            pair_idx <= pair_idx + 1'b1;
                            state    <= ST_REQ_PAIR;
                        end
                    end
                end

                default: begin
                    state <= ST_WAIT_LINK;
                end
            endcase
        end
    end
end

always @(*) begin
    builder_state_dbg = state;
    frame_id_dbg      = frame_id;
    line_id_dbg       = line_id;
    segment_id_dbg    = segment_id;
    pair_idx_dbg      = pair_idx;
    word_idx_dbg      = (state == ST_HEADER) ? {10'd0, header_idx} : {3'd0, pair_idx, (state == ST_PAY1)};
    packet_cnt_dbg    = packet_cnt;
    packet_word_cnt_dbg = packet_word_cnt;
    stream_active_dbg = stream_active;
end

endmodule
