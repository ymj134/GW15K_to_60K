`timescale 1ns / 1ps

// ============================================================================
// 720p B3-1-fix2-rx-capture-fix bridge
//
// Main function:
//   RoraLink RX receives native YUV422 pair packets from 15K.
//   Each payload word contains two pixels in YUV422 order.
//   This module converts each YUV422 pair to two RGB888_32 pixels,
//   packs four YUV422 payload words into one 256-bit DDR write beat,
//   writes a double-buffered framebuffer, then reads it to HDMI.
//
// Fix in this version:
//   RX capture decision is moved from header word1 to header word3.
//   Word1/word2 only latch/check header fields.  After word3 confirms
//   width/height, the next cycle is word4 payload, so capture_this_pkt is
//   opened in time without relying on early packet_magic_ok timing.
// ============================================================================

module roralink_video_to_ddr_hdmi_b2 #(
    parameter integer AXI_ADDR_WIDTH = 29,
    parameter integer AXI_DATA_WIDTH = 256,
    parameter integer AXI_ID_WIDTH   = 4,
    parameter integer H_RES          = 1280,
    parameter integer V_RES          = 720,
    parameter integer BURST_BEATS    = 64
)(
    input  wire                         axi_clk,
    input  wire                         axi_rst,

    input  wire                         pixel_clk,
    input  wire                         pixel_rst,
    input  wire                         display_de,
    input  wire                         display_frame_start,
    output wire [31:0]                  pixel_word,

    input  wire                         rx_clk,
    input  wire                         rx_rst_async,
    input  wire                         rx_channel_up,
    input  wire [31:0]                  rx_user_data,
    input  wire [3:0]                   rx_user_strb,
    input  wire                         rx_user_valid,
    input  wire                         rx_user_last,
    input  wire                         rx_crc_valid,
    input  wire                         rx_crc_pass_fail_n,
    input  wire                         rx_hard_err,
    input  wire                         rx_soft_err,
    input  wire                         rx_frame_err,

    output reg                          m_axi_awvalid,
    input  wire                         m_axi_awready,
    output wire [AXI_ID_WIDTH-1:0]      m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0]    m_axi_awaddr,
    output wire [7:0]                   m_axi_awlen,
    output wire [2:0]                   m_axi_awsize,
    output wire [1:0]                   m_axi_awburst,

    output reg                          m_axi_wvalid,
    input  wire                         m_axi_wready,
    output wire [AXI_DATA_WIDTH-1:0]    m_axi_wdata,
    output wire [AXI_DATA_WIDTH/8-1:0]  m_axi_wstrb,
    output wire                         m_axi_wlast,

    input  wire                         m_axi_bvalid,
    output reg                          m_axi_bready,
    input  wire [1:0]                   m_axi_bresp,
    input  wire [AXI_ID_WIDTH-1:0]      m_axi_bid,

    output reg                          m_axi_arvalid,
    input  wire                         m_axi_arready,
    output wire [AXI_ID_WIDTH-1:0]      m_axi_arid,
    output wire [AXI_ADDR_WIDTH-1:0]    m_axi_araddr,
    output wire [7:0]                   m_axi_arlen,
    output wire [2:0]                   m_axi_arsize,
    output wire [1:0]                   m_axi_arburst,

    input  wire                         m_axi_rvalid,
    output reg                          m_axi_rready,
    input  wire [AXI_DATA_WIDTH-1:0]    m_axi_rdata,
    input  wire [1:0]                   m_axi_rresp,
    input  wire [AXI_ID_WIDTH-1:0]      m_axi_rid,
    input  wire                         m_axi_rlast,

    output reg                          first_frame_written,
    output reg                          display_started,
    output reg                          error_seen,
    output reg                          underflow_seen,
    output reg  [3:0]                   state_dbg,
    output reg  [15:0]                  wr_burst_idx_dbg,
    output reg  [15:0]                  rd_burst_idx_dbg,
    output wire [15:0]                  rd_fifo_wr_count_dbg,
    output wire [15:0]                  rd_fifo_rd_count_dbg,
    output wire [15:0]                  wr_fifo_wr_count_dbg,
    output wire [15:0]                  wr_fifo_rd_count_dbg,
    output wire                         db_wr_buf_sel_dbg,
    output wire                         db_rd_buf_sel_dbg,
    output wire                         db_pending_valid_dbg,
    output wire                         db_pending_buf_sel_dbg,

    output reg  [31:0]                  rx_packet_cnt_dbg,
    output reg  [31:0]                  rx_crc_pass_cnt_dbg,
    output reg  [15:0]                  rx_good_seg_cnt_dbg,
    output reg                          rx_test_pass,
    output wire                         rx_any_err_seen,
    output wire [11:0]                  rx_word_idx_dbg,
    output wire [10:0]                  rx_cur_line_id_dbg,
    output wire [4:0]                   rx_cur_seg_id_dbg,
    output wire                         rx_capture_active_dbg,
    output wire                         rx_payload_accept_dbg,
    output wire                         rx_wr_fifo_wren_dbg,
    output wire                         rx_header_err_seen_dbg,
    output wire                         rx_last_err_seen_dbg,
    output wire                         rx_crc_err_seen_dbg,
    output wire                         rx_overrun_err_seen_dbg,
    output wire                         rx_frame_accept_allowed_dbg,
    output wire                         rx_drop_frame_active_dbg
);

    // ------------------------------------------------------------------------
    // Packet constants
    // ------------------------------------------------------------------------
    localparam [31:0] MAGIC              = 32'hA55A_6002;
    localparam [7:0]  FORMAT_YUV422_PAIR = 8'h02;
    localparam [11:0] H12                = 12'd1280;
    localparam [11:0] V12                = 12'd720;
    localparam [10:0] LAST_LINE          = 11'd719;
    localparam [4:0]  LAST_SEG           = 5'd4;
    localparam [11:0] SEG_PAYLOAD_WORDS_FULL = 12'd128; // 128 YUV422 words = 256 pixels
    localparam [11:0] SEG_PAYLOAD_WORDS_LAST = 12'd128;
    localparam [11:0] HEADER_WORDS       = 12'd4;
    localparam [15:0] STABLE_DELAY_MAX   = 16'h3FFF;
    localparam [15:0] PASS_SEG_TH        = 16'd512;

    function [11:0] segment_x_base;
        input [4:0] s;
        begin
            segment_x_base = {s[3:0], 8'h00}; // seg * 256
        end
    endfunction

    function [11:0] segment_payload_words;
        input [4:0] s;
        begin
            segment_payload_words = (s == LAST_SEG) ? SEG_PAYLOAD_WORDS_LAST : SEG_PAYLOAD_WORDS_FULL;
        end
    endfunction

    // ------------------------------------------------------------------------
    // RX reset / channel-up stable gate
    // ------------------------------------------------------------------------
    reg [3:0] rx_rst_shr = 4'hF;
    always @(posedge rx_clk or posedge rx_rst_async) begin
        if (rx_rst_async)
            rx_rst_shr <= 4'hF;
        else
            rx_rst_shr <= {rx_rst_shr[2:0], 1'b0};
    end

    wire rx_rst_base = rx_rst_shr[3];

    reg [15:0] stable_cnt  = 16'd0;
    reg        check_enable = 1'b0;

    always @(posedge rx_clk or posedge rx_rst_base) begin
        if (rx_rst_base) begin
            stable_cnt   <= 16'd0;
            check_enable <= 1'b0;
        end else begin
            if (!rx_channel_up) begin
                stable_cnt   <= 16'd0;
                check_enable <= 1'b0;
            end else if (!check_enable) begin
                if (stable_cnt == STABLE_DELAY_MAX)
                    check_enable <= 1'b1;
                else
                    stable_cnt <= stable_cnt + 1'b1;
            end
        end
    end

    wire rx_active = check_enable && rx_channel_up && !rx_rst_base;
    wire rx_fire   = rx_active && rx_user_valid;

    // ------------------------------------------------------------------------
    // Header field views
    // ------------------------------------------------------------------------
    wire [15:0] w1_frame         = rx_user_data[31:16];
    wire [10:0] w1_line          = rx_user_data[15:5];
    wire [4:0]  w1_seg           = rx_user_data[4:0];
    wire [7:0]  w2_format        = rx_user_data[31:24];
    wire [11:0] w2_x_base        = rx_user_data[23:12];
    wire [11:0] w2_payload_words = rx_user_data[11:0];
    wire [11:0] w3_width         = rx_user_data[23:12];
    wire [11:0] w3_height        = rx_user_data[11:0];

    // ------------------------------------------------------------------------
    // RX write FIFO: 256-bit rx_clk -> axi_clk
    // ------------------------------------------------------------------------
    wire        wr_fifo_full;
    wire        wr_fifo_empty;
    wire        wr_fifo_aempty;
    wire        wr_fifo_afull;
    wire [255:0] wr_fifo_q;
    wire [11:0]  wr_fifo_wnum;
    wire [11:0]  wr_fifo_rnum;
    reg          wr_fifo_rden;

    // ------------------------------------------------------------------------
    // AXI-domain free-buffer level synchronized into rx_clk
    // ------------------------------------------------------------------------
    wire capture_frame_available_axi;
    reg [2:0] capture_frame_available_rx_sync = 3'b000;

    always @(posedge rx_clk or posedge rx_rst_base) begin
        if (rx_rst_base)
            capture_frame_available_rx_sync <= 3'b000;
        else
            capture_frame_available_rx_sync <= {capture_frame_available_rx_sync[1:0], capture_frame_available_axi};
    end

    wire rx_wr_fifo_start_room = !wr_fifo_full && !wr_fifo_afull && (wr_fifo_wnum <= 12'd64);
    wire rx_can_start_frame    = capture_frame_available_rx_sync[2] && rx_wr_fifo_start_room;

    // ------------------------------------------------------------------------
    // RX depacketizer
    // ------------------------------------------------------------------------
    reg        rx_aligned          = 1'b0;
    reg        capture_active      = 1'b0;
    reg        capture_this_pkt    = 1'b0;
    reg        packet_magic_ok     = 1'b0;
    reg        packet_header_ok    = 1'b0;
    reg [11:0] word_idx            = 12'd0;

    reg [15:0] cur_frame_id        = 16'd0;
    reg [10:0] cur_line_id         = 11'd0;
    reg [4:0]  cur_seg_id          = 5'd0;
    reg [11:0] cur_x_base          = 12'd0;
    reg [11:0] cur_payload_words   = 12'd0;

    reg [255:0] rx_pack_reg        = 256'd0;
    reg [255:0] rx_pack_next;
    reg [255:0] rx_fifo_wdata      = 256'd0;
    reg         rx_fifo_wren       = 1'b0;

    reg [31:0] rx_packet_cnt       = 32'd0;
    reg [31:0] rx_crc_pass_cnt     = 32'd0;
    reg [15:0] rx_good_seg_cnt     = 16'd0;
    reg [31:0] rx_valid_cnt        = 32'd0;

    reg header_err_seen            = 1'b0;
    reg last_err_seen              = 1'b0;
    reg crc_err_seen               = 1'b0;
    reg strb_err_seen              = 1'b0;
    reg overrun_err_seen           = 1'b0;
    reg drop_frame_active          = 1'b0;

    wire [11:0] payload_pos    = word_idx - HEADER_WORDS;
    wire        payload_region = (word_idx >= HEADER_WORDS) &&
                                 (word_idx < (HEADER_WORDS + cur_payload_words));
    wire        exp_last_now   = rx_aligned &&
                                 (word_idx == (HEADER_WORDS + cur_payload_words - 12'd1));
    wire        payload_accept = rx_fire && payload_region && capture_this_pkt && packet_header_ok;

    assign rx_any_err_seen = header_err_seen | last_err_seen | crc_err_seen | strb_err_seen | overrun_err_seen;

    // ------------------------------------------------------------------------
    // YUV422 pair -> two RGB888_32 pixels
    // Byte order matches the verified 15K MIPI_YUV422toRGB888 path:
    //   yuv[7:0]   = U
    //   yuv[15:8]  = Y0
    //   yuv[23:16] = V
    //   yuv[31:24] = Y1
    // ------------------------------------------------------------------------
    function [7:0] clamp_rgb;
        input [19:0] v;
        begin
            clamp_rgb = v[10] ? 8'd0 : ((v[9:0] > 10'd255) ? 8'd255 : v[7:0]);
        end
    endfunction

    function [63:0] yuv422_to_rgb64;
        input [31:0] yuv;
        reg [19:0] Y0, Y1, U0, U1, V0, V1;
        reg [19:0] r0, g0, b0;
        reg [19:0] r1, g1, b1;
        reg [7:0]  p0r, p0g, p0b;
        reg [7:0]  p1r, p1g, p1b;
        begin
            Y0 = yuv[15:8]  * 20'd596;
            Y1 = yuv[31:24] * 20'd596;
            U0 = yuv[7:0]   * 20'd200;
            U1 = yuv[7:0]   * 20'd1033;
            V0 = yuv[23:16] * 20'd817;
            V1 = yuv[23:16] * 20'd416;

            r0 = (Y0 + V0 - 20'd114131) >> 9;
            g0 = (Y0 - U0 - V1 + 20'd69370) >> 9;
            b0 = (Y0 + U1 - 20'd141787) >> 9;

            r1 = (Y1 + V0 - 20'd114131) >> 9;
            g1 = (Y1 - U0 - V1 + 20'd69370) >> 9;
            b1 = (Y1 + U1 - 20'd141787) >> 9;

            p0r = clamp_rgb(r0);
            p0g = clamp_rgb(g0);
            p0b = clamp_rgb(b0);
            p1r = clamp_rgb(r1);
            p1g = clamp_rgb(g1);
            p1b = clamp_rgb(b1);

            // Lower 32 bits are earlier pixel so Video_FIFO_256to32 reads
            // pixel0 first, then pixel1.
            yuv422_to_rgb64 = {8'h00, p1r, p1g, p1b, 8'h00, p0r, p0g, p0b};
        end
    endfunction

    wire [63:0] rx_rgb_pair64 = yuv422_to_rgb64(rx_user_data);

    always @* begin
        rx_pack_next = rx_pack_reg;
        case (payload_pos[1:0])
            2'd0: rx_pack_next[63:0]    = rx_rgb_pair64;
            2'd1: rx_pack_next[127:64]  = rx_rgb_pair64;
            2'd2: rx_pack_next[191:128] = rx_rgb_pair64;
            2'd3: rx_pack_next[255:192] = rx_rgb_pair64;
            default: ;
        endcase
    end

    // ------------------------------------------------------------------------
    // RX packet parser
    // ------------------------------------------------------------------------
    always @(posedge rx_clk or posedge rx_rst_base) begin
        if (rx_rst_base) begin
            rx_aligned          <= 1'b0;
            capture_active      <= 1'b0;
            capture_this_pkt    <= 1'b0;
            packet_magic_ok     <= 1'b0;
            packet_header_ok    <= 1'b0;
            word_idx            <= 12'd0;
            cur_frame_id        <= 16'd0;
            cur_line_id         <= 11'd0;
            cur_seg_id          <= 5'd0;
            cur_x_base          <= 12'd0;
            cur_payload_words   <= 12'd0;
            rx_pack_reg         <= 256'd0;
            rx_fifo_wdata       <= 256'd0;
            rx_fifo_wren        <= 1'b0;
            rx_packet_cnt       <= 32'd0;
            rx_crc_pass_cnt     <= 32'd0;
            rx_good_seg_cnt     <= 16'd0;
            rx_valid_cnt        <= 32'd0;
            rx_packet_cnt_dbg   <= 32'd0;
            rx_crc_pass_cnt_dbg <= 32'd0;
            rx_good_seg_cnt_dbg <= 16'd0;
            rx_test_pass        <= 1'b0;
            header_err_seen     <= 1'b0;
            last_err_seen       <= 1'b0;
            crc_err_seen        <= 1'b0;
            strb_err_seen       <= 1'b0;
            overrun_err_seen    <= 1'b0;
            drop_frame_active   <= 1'b0;
        end else begin
            rx_fifo_wren <= 1'b0;

            if (!rx_active) begin
                rx_aligned       <= 1'b0;
                capture_active   <= 1'b0;
                capture_this_pkt <= 1'b0;
                drop_frame_active<= 1'b0;
                word_idx         <= 12'd0;
                rx_test_pass     <= 1'b0;
            end else begin
                if (rx_crc_valid) begin
                    if (rx_crc_pass_fail_n) begin
                        rx_crc_pass_cnt     <= rx_crc_pass_cnt + 1'b1;
                        rx_crc_pass_cnt_dbg <= rx_crc_pass_cnt + 1'b1;
                    end else begin
                        crc_err_seen <= 1'b1;
                    end
                end

                if (rx_fire) begin
                    rx_valid_cnt <= rx_valid_cnt + 1'b1;

                    if (!rx_aligned) begin
                        // Throw away the current partial packet/frame after startup.
                        if (rx_user_last) begin
                            rx_aligned <= 1'b1;
                            word_idx   <= 12'd0;
                        end
                    end else begin
                        // Header word0: MAGIC.  Start every packet with capture disabled;
                        // word3 will decide whether payload should be accepted.
                        if (word_idx == 12'd0) begin
                            packet_magic_ok  <= (rx_user_data == MAGIC);
                            packet_header_ok <= (rx_user_data == MAGIC);
                            capture_this_pkt <= 1'b0;
                            rx_pack_reg      <= 256'd0;
                            if (rx_user_data != MAGIC)
                                header_err_seen <= 1'b1;
                        end

                        // Header word1: frame/line/segment.  Latch only.
                        if (word_idx == 12'd1) begin
                            cur_frame_id <= w1_frame;
                            cur_line_id  <= w1_line;
                            cur_seg_id   <= w1_seg;

                            if ((w1_line > LAST_LINE) || (w1_seg > LAST_SEG)) begin
                                packet_header_ok <= 1'b0;
                                header_err_seen  <= 1'b1;
                            end
                        end

                        // Header word2: format/x_base/payload_words.
                        if (word_idx == 12'd2) begin
                            cur_x_base        <= w2_x_base;
                            cur_payload_words <= w2_payload_words;

                            if ((w2_format        != FORMAT_YUV422_PAIR) ||
                                (w2_x_base        != segment_x_base(cur_seg_id)) ||
                                (w2_payload_words != segment_payload_words(cur_seg_id))) begin
                                packet_header_ok <= 1'b0;
                                header_err_seen  <= 1'b1;
                            end
                        end

                        // Header word3: width/height.  After this cycle, word4 payload
                        // starts.  Therefore this is the correct place to open capture.
                        if (word_idx == 12'd3) begin
                            if ((w3_width != H12) || (w3_height != V12)) begin
                                packet_header_ok <= 1'b0;
                                header_err_seen  <= 1'b1;
                                capture_this_pkt <= 1'b0;
                            end else begin
                                if (packet_magic_ok && packet_header_ok) begin
                                    if ((cur_line_id == 11'd0) && (cur_seg_id == 5'd0)) begin
                                        if (rx_can_start_frame) begin
                                            capture_active    <= 1'b1;
                                            capture_this_pkt  <= 1'b1;
                                            drop_frame_active <= 1'b0;
                                            rx_pack_reg       <= 256'd0;
                                        end else begin
                                            capture_active    <= 1'b0;
                                            capture_this_pkt  <= 1'b0;
                                            drop_frame_active <= 1'b1;
                                            rx_pack_reg       <= 256'd0;
                                        end
                                    end else begin
                                        capture_this_pkt <= capture_active && !drop_frame_active;
                                    end
                                end else begin
                                    capture_this_pkt <= 1'b0;
                                end
                            end
                        end

                        if (rx_user_strb != 4'hF)
                            strb_err_seen <= 1'b1;

                        // Payload: YUV422-pair word -> two RGB888_32 pixels.
                        // Four payload words make one 256-bit write-FIFO beat.
                        if (payload_region) begin
                            if (payload_accept) begin
                                rx_pack_reg <= rx_pack_next;
                                if (payload_pos[1:0] == 2'd3) begin
                                    if (wr_fifo_full) begin
                                        overrun_err_seen  <= 1'b1;
                                        capture_active    <= 1'b0;
                                        capture_this_pkt  <= 1'b0;
                                        drop_frame_active <= 1'b1;
                                    end else begin
                                        rx_fifo_wdata <= rx_pack_next;
                                        rx_fifo_wren  <= 1'b1;
                                    end
                                end
                            end
                        end

                        if (rx_user_last) begin
                            rx_packet_cnt     <= rx_packet_cnt + 1'b1;
                            rx_packet_cnt_dbg <= rx_packet_cnt + 1'b1;

                            if (!exp_last_now)
                                last_err_seen <= 1'b1;

                            if (packet_header_ok && exp_last_now) begin
                                if (rx_good_seg_cnt != 16'hFFFF)
                                    rx_good_seg_cnt <= rx_good_seg_cnt + 1'b1;
                                rx_good_seg_cnt_dbg <= rx_good_seg_cnt + 1'b1;
                            end

                            // End of complete 720p frame: next frame must be accepted again.
                            if ((cur_line_id == LAST_LINE) && (cur_seg_id == LAST_SEG)) begin
                                capture_active    <= 1'b0;
                                capture_this_pkt  <= 1'b0;
                                drop_frame_active <= 1'b0;
                            end

                            word_idx <= 12'd0;
                        end else begin
                            if (exp_last_now)
                                last_err_seen <= 1'b1;
                            word_idx <= word_idx + 1'b1;
                        end
                    end
                end

                if ((rx_good_seg_cnt >= PASS_SEG_TH) &&
                    !overrun_err_seen && !strb_err_seen && !last_err_seen && !crc_err_seen) begin
                    rx_test_pass <= 1'b1;
                end
            end
        end
    end

    wire wr_fifo_reset = rx_rst_base | !check_enable;

    Video_WR_FIFO_256 u_video_wr_fifo_256 (
        .Data         (rx_fifo_wdata),
        .Reset        (wr_fifo_reset),
        .WrClk        (rx_clk),
        .RdClk        (axi_clk),
        .WrEn         (rx_fifo_wren),
        .RdEn         (wr_fifo_rden),
        .Wnum         (wr_fifo_wnum),
        .Rnum         (wr_fifo_rnum),
        .Almost_Empty (wr_fifo_aempty),
        .Almost_Full  (wr_fifo_afull),
        .Q            (wr_fifo_q),
        .Empty        (wr_fifo_empty),
        .Full         (wr_fifo_full)
    );

    assign rx_word_idx_dbg             = word_idx;
    assign rx_cur_line_id_dbg          = cur_line_id;
    assign rx_cur_seg_id_dbg           = cur_seg_id;
    assign rx_capture_active_dbg       = capture_active;
    assign rx_payload_accept_dbg       = payload_accept;
    assign rx_wr_fifo_wren_dbg         = rx_fifo_wren;
    assign rx_header_err_seen_dbg      = header_err_seen;
    assign rx_last_err_seen_dbg        = last_err_seen;
    assign rx_crc_err_seen_dbg         = crc_err_seen;
    assign rx_overrun_err_seen_dbg     = overrun_err_seen;
    assign rx_frame_accept_allowed_dbg = rx_can_start_frame;
    assign rx_drop_frame_active_dbg    = drop_frame_active;

    // ------------------------------------------------------------------------
    // DDR read-side FIFO: 256-bit axi_clk -> 32-bit pixel_clk
    // ------------------------------------------------------------------------
    reg        rd_stream_aligned;
    reg [7:0]  rd_fifo_rst_cnt;
    wire       rd_fifo_restart_active = (rd_fifo_rst_cnt != 8'd0);
    wire       rd_fifo_rst = axi_rst | !first_frame_written | !rd_stream_aligned | rd_fifo_restart_active;

    reg         rd_fifo_wren;
    wire [31:0] rd_fifo_q;
    wire        rd_fifo_empty;
    wire        rd_fifo_full;
    wire        rd_fifo_aempty;
    wire        rd_fifo_afull;
    wire [10:0] rd_fifo_wnum;
    wire [13:0] rd_fifo_rnum;
    wire        rd_fifo_rden;

    Video_FIFO_256to32 u_video_fifo_256to32 (
        .Data         (m_axi_rdata),
        .Reset        (rd_fifo_rst),
        .WrClk        (axi_clk),
        .RdClk        (pixel_clk),
        .WrEn         (rd_fifo_wren),
        .RdEn         (rd_fifo_rden),
        .Wnum         (rd_fifo_wnum),
        .Rnum         (rd_fifo_rnum),
        .Almost_Empty (rd_fifo_aempty),
        .Almost_Full  (rd_fifo_afull),
        .Q            (rd_fifo_q),
        .Empty        (rd_fifo_empty),
        .Full         (rd_fifo_full)
    );

    // ------------------------------------------------------------------------
    // AXI / framebuffer constants
    // ------------------------------------------------------------------------
    localparam integer PIXELS_PER_BEAT = AXI_DATA_WIDTH / 32;
    localparam integer FRAME_PIXELS    = H_RES * V_RES;
    localparam integer FRAME_BEATS     = FRAME_PIXELS / PIXELS_PER_BEAT;
    localparam integer TOTAL_BURSTS    = FRAME_BEATS / BURST_BEATS;
    localparam integer FRAME_BYTES     = FRAME_PIXELS * 4;

    localparam [AXI_ADDR_WIDTH-1:0] BUFFER0_BASE = {AXI_ADDR_WIDTH{1'b0}};
    localparam [AXI_ADDR_WIDTH-1:0] BUFFER1_BASE = FRAME_BYTES;
    localparam [7:0] AXI_LEN = BURST_BEATS - 1;

    localparam [13:0] FIFO_START_LEVEL      = 14'd4096;
    localparam [10:0] RD_FIFO_WR_SAFE_LEVEL = 11'd832;
    localparam        DEBUG_FREEZE_AFTER_FIRST_FRAME = 1'b0;

    localparam [3:0]
        ST_IDLE  = 4'd0,
        ST_WR_AW = 4'd1,
        ST_WR_W  = 4'd2,
        ST_WR_B  = 4'd3,
        ST_RD_AR = 4'd4,
        ST_RD_R  = 4'd5,
        ST_ERROR = 4'd15;

    assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}};
    assign m_axi_awlen   = AXI_LEN;
    assign m_axi_arlen   = AXI_LEN;
    assign m_axi_awsize  = 3'b101; // 32 bytes
    assign m_axi_arsize  = 3'b101;
    assign m_axi_awburst = 2'b01;
    assign m_axi_arburst = 2'b01;
    assign m_axi_wstrb   = {AXI_DATA_WIDTH/8{1'b1}};

    reg [3:0]  state;
    reg [15:0] wr_burst_idx;
    reg [15:0] rd_burst_idx;
    reg [7:0]  beat_idx;

    // Double-buffer control in AXI clock domain.
    // Buffer size for 720p RGB888_32: 1280*720*4 = 0x0038_4000.
    reg wr_buf_sel;
    reg rd_buf_sel;
    reg db_pending_valid;
    reg db_pending_buf_sel;

    assign db_wr_buf_sel_dbg      = wr_buf_sel;
    assign db_rd_buf_sel_dbg      = rd_buf_sel;
    assign db_pending_valid_dbg   = db_pending_valid;
    assign db_pending_buf_sel_dbg = db_pending_buf_sel;

    // ------------------------------------------------------------------------
    // HDMI frame_start CDC: pixel_clk -> axi_clk
    // ------------------------------------------------------------------------
    reg frame_start_toggle_pix;
    always @(posedge pixel_clk or posedge pixel_rst) begin
        if (pixel_rst)
            frame_start_toggle_pix <= 1'b0;
        else if (display_frame_start)
            frame_start_toggle_pix <= ~frame_start_toggle_pix;
    end

    reg [2:0] frame_start_sync_axi;
    wire hdmi_frame_start_axi = frame_start_sync_axi[2] ^ frame_start_sync_axi[1];
    reg  hdmi_frame_start_req;

    always @(posedge axi_clk) begin
        if (axi_rst)
            frame_start_sync_axi <= 3'b000;
        else
            frame_start_sync_axi <= {frame_start_sync_axi[1:0], frame_start_toggle_pix};
    end

    wire [AXI_ADDR_WIDTH-1:0] wr_burst_offset = {{(AXI_ADDR_WIDTH-27){1'b0}}, wr_burst_idx, 11'b0};
    wire [AXI_ADDR_WIDTH-1:0] rd_burst_offset = {{(AXI_ADDR_WIDTH-27){1'b0}}, rd_burst_idx, 11'b0};
    wire [AXI_ADDR_WIDTH-1:0] wr_buf_base     = wr_buf_sel ? BUFFER1_BASE : BUFFER0_BASE;
    wire [AXI_ADDR_WIDTH-1:0] rd_buf_base     = rd_buf_sel ? BUFFER1_BASE : BUFFER0_BASE;

    assign m_axi_awaddr = wr_buf_base + wr_burst_offset;
    assign m_axi_araddr = rd_buf_base + rd_burst_offset;
    assign m_axi_wdata  = wr_fifo_q;
    assign m_axi_wlast  = m_axi_wvalid && (beat_idx == AXI_LEN);

    localparam [11:0] WR_BURST_LEVEL = BURST_BEATS;
    wire wr_burst_available = (wr_fifo_rnum >= WR_BURST_LEVEL);
    wire rd_fifo_has_space  = (rd_fifo_wnum <= RD_FIFO_WR_SAFE_LEVEL);

    wire db_write_allowed = !DEBUG_FREEZE_AFTER_FIRST_FRAME &&
                            !db_pending_valid &&
                            (wr_buf_sel != rd_buf_sel);

    assign capture_frame_available_axi = !axi_rst && !error_seen &&
                                         (wr_burst_idx == 16'd0) &&
                                         ((!first_frame_written) || db_write_allowed);

    assign rd_fifo_wr_count_dbg = {5'd0, rd_fifo_wnum};
    assign rd_fifo_rd_count_dbg = {2'd0, rd_fifo_rnum};
    assign wr_fifo_wr_count_dbg = {4'd0, wr_fifo_wnum};
    assign wr_fifo_rd_count_dbg = {4'd0, wr_fifo_rnum};

    assign rd_fifo_rden = display_started && display_de && !rd_fifo_empty;
    assign pixel_word   = (display_started && display_de && rd_fifo_empty) ? 32'h00FF_0000 : rd_fifo_q;

    // ------------------------------------------------------------------------
    // Pixel-domain display start and underflow detect
    // ------------------------------------------------------------------------
    reg first_frame_written_p1;
    reg first_frame_written_p2;

    always @(posedge pixel_clk or posedge pixel_rst) begin
        if (pixel_rst) begin
            first_frame_written_p1 <= 1'b0;
            first_frame_written_p2 <= 1'b0;
            display_started        <= 1'b0;
            underflow_seen         <= 1'b0;
        end else begin
            first_frame_written_p1 <= first_frame_written;
            first_frame_written_p2 <= first_frame_written_p1;

            if (!display_started && first_frame_written_p2 &&
                (rd_fifo_rnum >= FIFO_START_LEVEL) && !display_de) begin
                display_started <= 1'b1;
            end

            if (display_started && display_de && rd_fifo_empty)
                underflow_seen <= 1'b1;
        end
    end

    // ------------------------------------------------------------------------
    // AXI write/read FSM
    // ------------------------------------------------------------------------
    always @(posedge axi_clk) begin
        if (axi_rst) begin
            state                <= ST_IDLE;
            m_axi_awvalid        <= 1'b0;
            m_axi_wvalid         <= 1'b0;
            m_axi_bready         <= 1'b0;
            m_axi_arvalid        <= 1'b0;
            m_axi_rready         <= 1'b0;
            wr_fifo_rden         <= 1'b0;
            rd_fifo_wren         <= 1'b0;
            first_frame_written  <= 1'b0;
            error_seen           <= 1'b0;
            wr_burst_idx         <= 16'd0;
            rd_burst_idx         <= 16'd0;
            beat_idx             <= 8'd0;
            wr_buf_sel           <= 1'b0;
            rd_buf_sel           <= 1'b0;
            db_pending_valid     <= 1'b0;
            db_pending_buf_sel   <= 1'b0;
            rd_stream_aligned    <= 1'b0;
            rd_fifo_rst_cnt      <= 8'd0;
            hdmi_frame_start_req <= 1'b0;
            state_dbg            <= ST_IDLE;
            wr_burst_idx_dbg     <= 16'd0;
            rd_burst_idx_dbg     <= 16'd0;
        end else begin
            wr_fifo_rden <= 1'b0;
            rd_fifo_wren <= 1'b0;

            if (rd_fifo_rst_cnt != 8'd0)
                rd_fifo_rst_cnt <= rd_fifo_rst_cnt - 1'b1;

            if (hdmi_frame_start_axi && first_frame_written)
                hdmi_frame_start_req <= 1'b1;

            state_dbg        <= state;
            wr_burst_idx_dbg <= wr_burst_idx;
            rd_burst_idx_dbg <= rd_burst_idx;

            case (state)
                ST_IDLE: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;

                    if (!first_frame_written) begin
                        // Fill buffer0 first. HDMI reader is disabled until this frame completes.
                        if (wr_burst_available) begin
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_WR_AW;
                        end
                    end else begin
                        // On every HDMI frame_start, restart DDR read stream from selected buffer.
                        if (hdmi_frame_start_req) begin
                            if (db_pending_valid) begin
                                rd_buf_sel       <= db_pending_buf_sel;
                                db_pending_valid <= 1'b0;
                            end
                            rd_burst_idx         <= 16'd0;
                            rd_stream_aligned    <= 1'b1;
                            rd_fifo_rst_cnt      <= 8'd128;
                            hdmi_frame_start_req <= 1'b0;
                        end else if (db_write_allowed && (wr_fifo_rnum >= 12'd1024) && wr_burst_available) begin
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_WR_AW;
                        end else if (rd_stream_aligned && !rd_fifo_restart_active && rd_fifo_has_space) begin
                            m_axi_arvalid <= 1'b1;
                            state         <= ST_RD_AR;
                        end else if (db_write_allowed && wr_burst_available) begin
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_WR_AW;
                        end
                    end
                end

                ST_WR_AW: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b1;
                        beat_idx      <= 8'd0;
                        state         <= ST_WR_W;
                    end
                end

                ST_WR_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        if (wr_fifo_empty) begin
                            error_seen    <= 1'b1;
                            m_axi_wvalid  <= 1'b0;
                            state         <= ST_ERROR;
                        end else begin
                            wr_fifo_rden <= 1'b1;
                            if (beat_idx == AXI_LEN) begin
                                m_axi_wvalid <= 1'b0;
                                m_axi_bready <= 1'b1;
                                state        <= ST_WR_B;
                            end else begin
                                beat_idx <= beat_idx + 1'b1;
                            end
                        end
                    end
                end

                ST_WR_B: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        if (m_axi_bresp != 2'b00) begin
                            error_seen <= 1'b1;
                            state      <= ST_ERROR;
                        end else begin
                            if (wr_burst_idx == TOTAL_BURSTS - 1) begin
                                wr_burst_idx <= 16'd0;
                                if (!first_frame_written) begin
                                    first_frame_written <= 1'b1;
                                    rd_buf_sel          <= wr_buf_sel;
                                    wr_buf_sel          <= ~wr_buf_sel;
                                    db_pending_valid    <= 1'b0;
                                end else begin
                                    db_pending_buf_sel <= wr_buf_sel;
                                    db_pending_valid   <= 1'b1;
                                    wr_buf_sel         <= ~wr_buf_sel;
                                end
                            end else begin
                                wr_burst_idx <= wr_burst_idx + 1'b1;
                            end
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_RD_AR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        beat_idx      <= 8'd0;
                        state         <= ST_RD_R;
                    end
                end

                ST_RD_R: begin
                    m_axi_rready <= 1'b1;
                    if (m_axi_rvalid) begin
                        if ((m_axi_rresp != 2'b00) || rd_fifo_full) begin
                            error_seen   <= 1'b1;
                            m_axi_rready <= 1'b0;
                            state        <= ST_ERROR;
                        end else begin
                            rd_fifo_wren <= 1'b1;
                            if (m_axi_rlast) begin
                                m_axi_rready <= 1'b0;
                                if (rd_burst_idx == TOTAL_BURSTS - 1)
                                    rd_burst_idx <= 16'd0;
                                else
                                    rd_burst_idx <= rd_burst_idx + 1'b1;
                                state <= ST_IDLE;
                            end
                        end
                    end
                end

                ST_ERROR: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_bready  <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b0;
                    wr_fifo_rden  <= 1'b0;
                    rd_fifo_wren  <= 1'b0;
                    error_seen    <= 1'b1;
                end

                default: begin
                    state <= ST_ERROR;
                end
            endcase
        end
    end

endmodule
