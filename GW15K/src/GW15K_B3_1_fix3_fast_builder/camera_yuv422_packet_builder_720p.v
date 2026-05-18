// ============================================================================
// Camera cropped YUV422-pair stream -> RoraLink segmented packet FIFO writer
// B3-1-fix3: fast 1-pair/clk payload writer
// ----------------------------------------------------------------------------
// Clock domain: camera byte_clk.
//
// Why this version:
//   Previous fix2 used a registered pair FIFO and consumed one pair in three
//   cycles: REQ -> LOAD -> PAYLOAD. The crop stream can produce one YUV422 pair
//   every byte_clk, so the internal pair FIFO could overflow. This version uses
//   a same-clock FWFT-style elastic FIFO and writes one payload word per clock
//   whenever packet FIFO space is available.
//
// Input crop stream:
//   crop_yuv422 = two pixels in one YUV422 32-bit word.
//
// Output packet FIFO word:
//   packet_fifo_wr_data = {last, data[31:0]}
//
// Packet format:
//   word0   : 32'hA55A_6002
//   word1   : {frame_id[15:0], line_id[10:0], segment_id[4:0]}
//   word2   : {format[7:0], x_base[11:0], payload_words[11:0]}
//   word3   : {flags[7:0], width[11:0], height[11:0]}
//   payload : 128 YUV422-pair words per 256-pixel segment.
// ============================================================================
`timescale 1ns / 1ps

module camera_yuv422_packet_builder_720p #(
    parameter [31:0] MAGIC              = 32'hA55A_6002,
    parameter [7:0]  FORMAT_YUV422_PAIR = 8'h02,
    parameter [11:0] H_RES              = 12'd1280,
    parameter [11:0] V_RES              = 12'd720,
    parameter [10:0] LAST_LINE          = 11'd719,
    parameter [4:0]  LAST_SEG           = 5'd4,
    parameter [11:0] SEG_PIXELS         = 12'd256,
    parameter integer PAIR_FIFO_ADDR_WIDTH = 9,
    parameter [15:0] START_PREFILL_PAIRS = 16'd64   // kept for interface compatibility; unused in fast mode
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

    localparam [3:0] ST_WAIT_LINK  = 4'd0;
    localparam [3:0] ST_WAIT_FRAME = 4'd1;
    localparam [3:0] ST_HEADER     = 4'd2;
    localparam [3:0] ST_PAYLOAD    = 4'd3;

    localparam [7:0] PAIRS_PER_SEG = SEG_PIXELS[8:1];   // 256 pixels -> 128 YUV422-pair words

    // ------------------------------------------------------------------------
    // Same-clock FWFT-style elastic FIFO for cropped YUV422 pairs.
    // This FIFO only absorbs header insertion and packet FIFO backpressure.
    // ------------------------------------------------------------------------
    localparam integer PAIR_FIFO_DEPTH = (1 << PAIR_FIFO_ADDR_WIDTH);

    reg [31:0] pair_mem [0:PAIR_FIFO_DEPTH-1];
    reg [PAIR_FIFO_ADDR_WIDTH:0] pair_wr_ptr;
    reg [PAIR_FIFO_ADDR_WIDTH:0] pair_rd_ptr;

    wire [PAIR_FIFO_ADDR_WIDTH:0] pair_count = pair_wr_ptr - pair_rd_ptr;
    wire pair_fifo_empty = (pair_count == {PAIR_FIFO_ADDR_WIDTH+1{1'b0}});
    wire pair_fifo_full  = (pair_count == {1'b1, {PAIR_FIFO_ADDR_WIDTH{1'b0}}});
    wire [31:0] pair_fifo_q = pair_mem[pair_rd_ptr[PAIR_FIFO_ADDR_WIDTH-1:0]];

    reg pair_fifo_rd_en;

    assign pair_fifo_full_dbg     = pair_fifo_full;
    assign pair_fifo_empty_dbg    = pair_fifo_empty;
    assign pair_fifo_wr_count_dbg = {{(16-(PAIR_FIFO_ADDR_WIDTH+1)){1'b0}}, pair_count};
    assign pair_fifo_rd_count_dbg = {{(16-(PAIR_FIFO_ADDR_WIDTH+1)){1'b0}}, pair_count};
    assign pair_fifo_rd_en_dbg    = pair_fifo_rd_en;

    // ------------------------------------------------------------------------
    // Frame capture gate.
    // Do not gate new frame start with Almost_Full. Some Gowin FIFO AFULL
    // thresholds are intentionally conservative; using it here can permanently
    // block capture even while the TX side is draining normally.
    // ------------------------------------------------------------------------
    reg frame_capture_active;
    reg frame_output_active;

    wire frame_can_start = link_active &&
                           !frame_capture_active &&
                           !frame_output_active &&
                           !packet_fifo_full;

    wire start_frame      = crop_valid && crop_sof && frame_can_start;
    wire drop_frame_start = crop_valid && crop_sof && link_active && !frame_can_start;
    wire capture_this_pair = crop_valid && (frame_capture_active || start_frame);

    // ------------------------------------------------------------------------
    // Packet state.
    // ------------------------------------------------------------------------
    reg [3:0]  state;
    reg [15:0] frame_id;
    reg [10:0] line_id;
    reg [4:0]  segment_id;
    reg [1:0]  header_idx;
    reg [7:0]  pair_idx;
    reg [31:0] packet_cnt;
    reg [31:0] packet_word_cnt;

    wire [11:0] payload_words = {4'd0, PAIRS_PER_SEG};
    wire [11:0] x_base        = {segment_id, 8'd0};  // segment_id * 256 pixels

    wire [31:0] header_word =
        (header_idx == 2'd0) ? MAGIC :
        (header_idx == 2'd1) ? {frame_id, line_id, segment_id} :
        (header_idx == 2'd2) ? {FORMAT_YUV422_PAIR, x_base, payload_words} :
                               {8'h00, H_RES, V_RES};

    // Current-cycle FIFO operations, generated inside sequential block but also
    // used for pointer update in the same clock edge.
    reg pair_fifo_wr_do;
    reg pair_fifo_rd_do;

    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pair_wr_ptr <= {PAIR_FIFO_ADDR_WIDTH+1{1'b0}};
            pair_rd_ptr <= {PAIR_FIFO_ADDR_WIDTH+1{1'b0}};

            state <= ST_WAIT_LINK;
            frame_capture_active <= 1'b0;
            frame_output_active  <= 1'b0;
            frame_id   <= 16'd0;
            line_id    <= 11'd0;
            segment_id <= 5'd0;
            header_idx <= 2'd0;
            pair_idx   <= 8'd0;

            pair_fifo_rd_en <= 1'b0;
            packet_fifo_wr_en   <= 1'b0;
            packet_fifo_wr_data <= 33'd0;

            packet_cnt      <= 32'd0;
            packet_word_cnt <= 32'd0;

            sof_seen_dbg                 <= 1'b0;
            packet_wr_seen_dbg           <= 1'b0;
            packet_last_seen_dbg         <= 1'b0;
            pair_fifo_overflow_seen_dbg  <= 1'b0;
            pair_fifo_empty_seen_dbg     <= 1'b0;
            packet_fifo_full_seen_dbg    <= 1'b0;
            packet_fifo_afull_seen_dbg   <= 1'b0;
            drop_frame_seen_dbg          <= 1'b0;
        end else begin
            // Defaults.
            pair_fifo_wr_do   = 1'b0;
            pair_fifo_rd_do   = 1'b0;
            pair_fifo_rd_en   <= 1'b0;
            packet_fifo_wr_en <= 1'b0;

            if (packet_fifo_full)
                packet_fifo_full_seen_dbg <= 1'b1;
            if (packet_fifo_almost_full)
                packet_fifo_afull_seen_dbg <= 1'b1;

            if (!link_active) begin
                // Drop all partial state when link is down.
                pair_wr_ptr <= {PAIR_FIFO_ADDR_WIDTH+1{1'b0}};
                pair_rd_ptr <= {PAIR_FIFO_ADDR_WIDTH+1{1'b0}};
                state <= ST_WAIT_LINK;
                frame_capture_active <= 1'b0;
                frame_output_active  <= 1'b0;
                header_idx <= 2'd0;
                pair_idx   <= 8'd0;
                line_id    <= 11'd0;
                segment_id <= 5'd0;
            end else begin
                // Capture input cropped pairs into the elastic FIFO.
                if (capture_this_pair) begin
                    if (!pair_fifo_full) begin
                        pair_fifo_wr_do = 1'b1;
                        pair_mem[pair_wr_ptr[PAIR_FIFO_ADDR_WIDTH-1:0]] <= crop_yuv422;
                    end else begin
                        pair_fifo_overflow_seen_dbg <= 1'b1;
                        // Abort current frame to avoid mixing frame fragments.
                        frame_capture_active <= 1'b0;
                        frame_output_active  <= 1'b0;
                        state <= ST_WAIT_FRAME;
                        pair_wr_ptr <= {PAIR_FIFO_ADDR_WIDTH+1{1'b0}};
                        pair_rd_ptr <= {PAIR_FIFO_ADDR_WIDTH+1{1'b0}};
                    end
                end

                if (start_frame) begin
                    frame_capture_active <= 1'b1;
                    frame_output_active  <= 1'b1;
                    sof_seen_dbg <= 1'b1;
                    line_id    <= 11'd0;
                    segment_id <= 5'd0;
                    header_idx <= 2'd0;
                    pair_idx   <= 8'd0;
                    if (state == ST_WAIT_LINK || state == ST_WAIT_FRAME)
                        state <= ST_HEADER;
                end else if (drop_frame_start) begin
                    drop_frame_seen_dbg <= 1'b1;
                end

                if (capture_this_pair && crop_eof)
                    frame_capture_active <= 1'b0;

                // Output one complete segmented packet stream.
                case (state)
                    ST_WAIT_LINK: begin
                        if (link_active)
                            state <= ST_WAIT_FRAME;
                    end

                    ST_WAIT_FRAME: begin
                        // Start is handled by start_frame above.
                    end

                    ST_HEADER: begin
                        if (!packet_fifo_full && frame_output_active) begin
                            packet_fifo_wr_en   <= 1'b1;
                            packet_fifo_wr_data <= {1'b0, header_word};
                            packet_wr_seen_dbg  <= 1'b1;
                            packet_word_cnt     <= packet_word_cnt + 1'b1;

                            if (header_idx == 2'd3) begin
                                header_idx <= 2'd0;
                                pair_idx   <= 8'd0;
                                state      <= ST_PAYLOAD;
                            end else begin
                                header_idx <= header_idx + 1'b1;
                            end
                        end
                    end

                    ST_PAYLOAD: begin
                        if (packet_fifo_full) begin
                            // Stall; pair FIFO absorbs camera data.
                        end else if (!pair_fifo_empty) begin
                            packet_fifo_wr_en   <= 1'b1;
                            packet_fifo_wr_data <= {(pair_idx == PAIRS_PER_SEG - 8'd1), pair_fifo_q};
                            packet_wr_seen_dbg  <= 1'b1;
                            packet_word_cnt     <= packet_word_cnt + 1'b1;

                            pair_fifo_rd_do = 1'b1;
                            pair_fifo_rd_en <= 1'b1;

                            if (pair_idx == PAIRS_PER_SEG - 8'd1) begin
                                packet_last_seen_dbg <= 1'b1;
                                packet_cnt <= packet_cnt + 1'b1;
                                pair_idx <= 8'd0;

                                if (segment_id == LAST_SEG) begin
                                    segment_id <= 5'd0;
                                    if (line_id == LAST_LINE) begin
                                        line_id <= 11'd0;
                                        frame_id <= frame_id + 1'b1;
                                        frame_output_active <= 1'b0;
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
                            end
                        end else begin
                            pair_fifo_empty_seen_dbg <= 1'b1;
                            // Stall here until the camera crop stream supplies
                            // the next pair. This is normal around line starts.
                        end
                    end

                    default: begin
                        state <= ST_WAIT_LINK;
                    end
                endcase

                // FIFO pointer updates. Do this after state logic, unless the
                // overflow path above has force-cleared the FIFO. If overflow was
                // set this cycle, the force clear value wins through the state
                // assignment above by not applying pointer increments.
                if (!(capture_this_pair && pair_fifo_full)) begin
                    if (pair_fifo_wr_do)
                        pair_wr_ptr <= pair_wr_ptr + 1'b1;
                    if (pair_fifo_rd_do)
                        pair_rd_ptr <= pair_rd_ptr + 1'b1;
                end
            end
        end
    end

    always @(*) begin
        builder_state_dbg  = state;
        frame_id_dbg       = frame_id;
        line_id_dbg        = line_id;
        segment_id_dbg     = segment_id;
        pair_idx_dbg       = pair_idx;
        word_idx_dbg       = (state == ST_HEADER) ? {10'd0, header_idx} : {4'd0, pair_idx};
        packet_cnt_dbg     = packet_cnt;
        packet_word_cnt_dbg= packet_word_cnt;
        frame_active_dbg   = frame_capture_active | frame_output_active;
    end

endmodule
