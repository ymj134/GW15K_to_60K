// ============================================================================
// Camera cropped YUV422-pair stream -> RoraLink segmented packet FIFO writer
// ----------------------------------------------------------------------------
// Clock domain: camera byte_clk.
//
// Input crop stream:
//   crop_yuv422 = two pixels in one YUV422 32-bit word.
//
// Output packet FIFO word:
//   packet_fifo_wr_data = {last, data[31:0]}
//
// Packet format:
//   word0 : 32'hA55A_6002
//   word1 : {frame_id[15:0], line_id[10:0], segment_id[4:0]}
//   word2 : {format[7:0], x_base[11:0], payload_words[11:0]}
//   word3 : {flags[7:0], width[11:0], height[11:0]}
//   payload: 128 YUV422-pair words per 256-pixel segment.
//
// FORMAT_YUV422_PAIR = 8'h02. The matched 60K B3-1-fix2 receiver converts
// YUV422 pairs to RGB888_32 before writing DDR.
// ============================================================================

`timescale 1ns / 1ps

module camera_yuv422_packet_builder_720p #(
    parameter [31:0] MAGIC                 = 32'hA55A_6002,
    parameter [7:0]  FORMAT_YUV422_PAIR    = 8'h02,
    parameter [11:0] H_RES                 = 12'd1280,
    parameter [11:0] V_RES                 = 12'd720,
    parameter [10:0] LAST_LINE             = 11'd719,
    parameter [4:0]  LAST_SEG              = 5'd4,
    parameter [11:0] SEG_PIXELS            = 12'd256,
    parameter integer PAIR_FIFO_ADDR_WIDTH = 9,
    parameter [15:0] START_PREFILL_PAIRS   = 16'd64
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        link_active,

    input  wire        crop_valid,
    input  wire        crop_sof,
    input  wire        crop_eof,
    input  wire [31:0] crop_yuv422,

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
    output reg         frame_active_dbg,
    output reg         sof_seen_dbg,
    output reg         packet_wr_seen_dbg,
    output reg         packet_last_seen_dbg,
    output reg         pair_fifo_overflow_seen_dbg,
    output reg         pair_fifo_empty_seen_dbg,
    output reg         packet_fifo_full_seen_dbg,
    output reg         packet_fifo_afull_seen_dbg,
    output reg         drop_frame_seen_dbg
);

localparam [3:0] ST_WAIT_LINK    = 4'd0;
localparam [3:0] ST_WAIT_FRAME   = 4'd1;
localparam [3:0] ST_WAIT_PREFILL = 4'd2;
localparam [3:0] ST_HEADER       = 4'd3;
localparam [3:0] ST_REQ_PAIR     = 4'd4;
localparam [3:0] ST_LOAD_PAIR    = 4'd5;
localparam [3:0] ST_PAYLOAD      = 4'd6;

localparam [7:0] PAIRS_PER_SEG = SEG_PIXELS[8:1]; // 256 pixels -> 128 YUV422 pair words

wire pair_fifo_rst = rst | ~link_active;

reg        frame_capture_active;
reg        frame_output_active;
reg        frame_done_pending;
wire       frame_can_start = link_active && !frame_capture_active && !frame_output_active &&
                             !frame_done_pending && !packet_fifo_almost_full &&
                             !packet_fifo_full && pair_fifo_empty_dbg;
wire       start_frame = crop_valid && crop_sof && frame_can_start;
wire       drop_frame_start = crop_valid && crop_sof && link_active && !frame_can_start;


reg [3:0]  state;
reg [15:0] frame_id;
reg [10:0] line_id;
reg [4:0]  segment_id;
reg [1:0]  header_idx;
reg [7:0]  pair_idx;
reg [31:0] pair_buf;
reg [31:0] packet_cnt;
reg [31:0] packet_word_cnt;
reg        frame_output_done_pulse;

wire       pair_fifo_full;
wire       pair_fifo_empty;
wire [31:0] pair_fifo_rd_data;
wire [PAIR_FIFO_ADDR_WIDTH:0] pair_fifo_wr_count_raw;
wire [PAIR_FIFO_ADDR_WIDTH:0] pair_fifo_rd_count_raw;
reg        pair_fifo_rd_en;

wire pair_fifo_wr_en = crop_valid && (frame_capture_active || start_frame) && !pair_fifo_full;

always @(posedge clk or posedge pair_fifo_rst) begin
    if (pair_fifo_rst) begin
        frame_capture_active        <= 1'b0;
        frame_done_pending          <= 1'b0;
        sof_seen_dbg                <= 1'b0;
        pair_fifo_overflow_seen_dbg <= 1'b0;
        drop_frame_seen_dbg         <= 1'b0;
    end else begin
        if (start_frame) begin
            frame_capture_active <= 1'b1;
            frame_done_pending   <= 1'b0;
            sof_seen_dbg         <= 1'b1;
        end else if (drop_frame_start) begin
            drop_frame_seen_dbg <= 1'b1;
        end

        if (crop_valid && frame_capture_active && pair_fifo_full) begin
            pair_fifo_overflow_seen_dbg <= 1'b1;
            frame_capture_active        <= 1'b0;
            frame_done_pending          <= 1'b0;
        end else if (crop_valid && frame_capture_active && crop_eof) begin
            frame_capture_active <= 1'b0;
            frame_done_pending   <= 1'b1;
        end

        // Cleared by output FSM when the last segment of this frame has been emitted.
        if (frame_output_done_pulse)
            frame_done_pending <= 1'b0;
    end
end

async_fifo_gray #(
    .DATA_WIDTH (32),
    .ADDR_WIDTH (PAIR_FIFO_ADDR_WIDTH)
) u_yuv_pair_elastic_fifo (
    .rst      (pair_fifo_rst),
    .wr_clk   (clk),
    .wr_en    (pair_fifo_wr_en),
    .wr_data  (crop_yuv422),
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

wire [11:0] payload_words = {4'd0, PAIRS_PER_SEG};
wire [11:0] x_base        = {segment_id, 8'd0}; // segment_id * 256 pixels
wire [31:0] header_word =
    (header_idx == 2'd0) ? MAGIC :
    (header_idx == 2'd1) ? {frame_id, line_id, segment_id} :
    (header_idx == 2'd2) ? {FORMAT_YUV422_PAIR, x_base, payload_words} :
                            {8'h00, H_RES, V_RES};

wire prefill_ok = (pair_fifo_rd_count_dbg >= START_PREFILL_PAIRS) || frame_done_pending;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                      <= ST_WAIT_LINK;
        frame_output_active        <= 1'b0;
        frame_output_done_pulse    <= 1'b0;
        frame_id                   <= 16'd0;
        line_id                    <= 11'd0;
        segment_id                 <= 5'd0;
        header_idx                 <= 2'd0;
        pair_idx                   <= 8'd0;
        pair_buf                   <= 32'd0;
        pair_fifo_rd_en            <= 1'b0;
        packet_fifo_wr_en          <= 1'b0;
        packet_fifo_wr_data        <= 33'd0;
        packet_cnt                 <= 32'd0;
        packet_word_cnt            <= 32'd0;
        packet_wr_seen_dbg         <= 1'b0;
        packet_last_seen_dbg       <= 1'b0;
        pair_fifo_empty_seen_dbg   <= 1'b0;
        packet_fifo_full_seen_dbg  <= 1'b0;
        packet_fifo_afull_seen_dbg <= 1'b0;
    end else begin
        pair_fifo_rd_en         <= 1'b0;
        packet_fifo_wr_en       <= 1'b0;
        frame_output_done_pulse <= 1'b0;

        if (packet_fifo_full)
            packet_fifo_full_seen_dbg <= 1'b1;
        if (packet_fifo_almost_full)
            packet_fifo_afull_seen_dbg <= 1'b1;

        if (!link_active) begin
            state               <= ST_WAIT_LINK;
            frame_output_active <= 1'b0;
            header_idx          <= 2'd0;
            pair_idx            <= 8'd0;
            line_id             <= 11'd0;
            segment_id          <= 5'd0;
        end else begin
            case (state)
                ST_WAIT_LINK: begin
                    frame_output_active <= 1'b0;
                    header_idx <= 2'd0;
                    pair_idx   <= 8'd0;
                    if (link_active)
                        state <= ST_WAIT_FRAME;
                end

                ST_WAIT_FRAME: begin
                    header_idx <= 2'd0;
                    pair_idx   <= 8'd0;
                    if ((frame_capture_active || frame_done_pending) && prefill_ok) begin
                        frame_output_active <= 1'b1;
                        state <= ST_HEADER;
                    end
                end

                ST_WAIT_PREFILL: begin
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
                    state    <= ST_PAYLOAD;
                end

                ST_PAYLOAD: begin
                    if (!packet_fifo_full) begin
                        packet_fifo_wr_en   <= 1'b1;
                        packet_fifo_wr_data <= {(pair_idx == PAIRS_PER_SEG - 8'd1), pair_buf};
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
                                    frame_output_active     <= 1'b0;
                                    frame_output_done_pulse <= 1'b1;
                                    state <= ST_WAIT_FRAME;
                                end else begin
                                    line_id <= line_id + 1'b1;
                                    state <= ST_HEADER;
                                end
                            end else begin
                                segment_id <= segment_id + 1'b1;
                                state <= ST_HEADER;
                            end
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
    builder_state_dbg    = state;
    frame_id_dbg         = frame_id;
    line_id_dbg          = line_id;
    segment_id_dbg       = segment_id;
    pair_idx_dbg         = pair_idx;
    word_idx_dbg         = (state == ST_HEADER) ? {10'd0, header_idx} : {4'd0, pair_idx};
    packet_cnt_dbg       = packet_cnt;
    packet_word_cnt_dbg  = packet_word_cnt;
    frame_active_dbg     = frame_capture_active | frame_output_active | frame_done_pending;
end

endmodule
