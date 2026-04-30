`timescale 1ns / 1ps
// ============================================================================
// 60K RoraLink 8B10B RX-only video-line packet checker
// ----------------------------------------------------------------------------
// Target:
//   GW5AT-LV60PG484AC2/I1
//   RoraLink 8B10B RX-only Simplex, Framing, Timer BackChannel
//   1 lane, 32-bit user data, 6.25Gbps line rate, 125MHz refclk
//
// Expected packet format from 15K TX, one RoraLink frame per video line:
//   word0 : 32'hA55A_6001
//   word1 : {frame_id[15:0], 5'd0, line_id[10:0]}
//   word2 : {16'd1920, 16'd1080}
//   word3 : {8'h01, 8'h00, 16'd1920}   // RGB888, flags, payload words
//   word4~word1923 : {8'h00, rgb888[23:0]}
//
// LED:
//   O_led[0] = gt_pll_ok & lane_up & channel_up
//   O_led[1] = pass: ON, error: fast blink, waiting: slow blink
// ============================================================================

module top (
    input  wire       clk,      // board clock, expected 50MHz
    input  wire       rst_n,    // active-low reset
    output wire [1:0] O_led
);

    localparam [31:0] TOP_VERSION = 32'h60A2_3001;

    localparam integer H_RES          = 1920;
    localparam integer V_RES          = 1080;
    localparam integer HEADER_WORDS   = 4;
    localparam integer PAYLOAD_WORDS  = H_RES;
    localparam integer PACKET_WORDS   = HEADER_WORDS + PAYLOAD_WORDS;
    localparam [31:0]  MAGIC_WORD     = 32'hA55A_6001;
    localparam [15:0]  PASS_LINE_TH   = 16'd1024;

    // ------------------------------------------------------------------------
    // 50MHz reset sync for SerDes init/reset input
    // ------------------------------------------------------------------------
    reg [3:0] rst_sync_50m;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rst_sync_50m <= 4'b0000;
        else
            rst_sync_50m <= {rst_sync_50m[2:0], 1'b1};
    end
    wire rst_n_50m = rst_sync_50m[3];

    // ------------------------------------------------------------------------
    // RoraLink RX-only IP
    // ------------------------------------------------------------------------
    wire        link_reset;
    wire        rl_sys_reset;
    wire [31:0] user_rx_data;
    wire [3:0]  user_rx_strb;
    wire        user_rx_valid;
    wire        user_rx_last;
    wire        crc_pass_fail_n;
    wire        crc_valid;
    wire        hard_err;
    wire        soft_err;
    wire        frame_err;
    wire        channel_up;
    wire        lane_up;
    wire        rx_clk;
    wire        gt_pll_ok;
    wire        gt_rx_align_link;
    wire        gt_rx_pma_lock;
    wire        gt_rx_k_lock;

    SerDes_Top u_serdes_top (
        .RoraLink_8B10B_Top_link_reset_o        (link_reset),
        .RoraLink_8B10B_Top_sys_reset_o         (rl_sys_reset),
        .RoraLink_8B10B_Top_user_rx_data_o      (user_rx_data),
        .RoraLink_8B10B_Top_user_rx_strb_o      (user_rx_strb),
        .RoraLink_8B10B_Top_user_rx_valid_o     (user_rx_valid),
        .RoraLink_8B10B_Top_user_rx_last_o      (user_rx_last),
        .RoraLink_8B10B_Top_crc_pass_fail_n_o   (crc_pass_fail_n),
        .RoraLink_8B10B_Top_crc_valid_o         (crc_valid),
        .RoraLink_8B10B_Top_hard_err_o          (hard_err),
        .RoraLink_8B10B_Top_soft_err_o          (soft_err),
        .RoraLink_8B10B_Top_frame_err_o         (frame_err),
        .RoraLink_8B10B_Top_channel_up_o        (channel_up),
        .RoraLink_8B10B_Top_lane_up_o           (lane_up),
        .RoraLink_8B10B_Top_gt_pcs_rx_clk_o     (rx_clk),
        .RoraLink_8B10B_Top_gt_pll_lock_o       (gt_pll_ok),
        .RoraLink_8B10B_Top_gt_rx_align_link_o  (gt_rx_align_link),
        .RoraLink_8B10B_Top_gt_rx_pma_lock_o    (gt_rx_pma_lock),
        .RoraLink_8B10B_Top_gt_rx_k_lock_o      (gt_rx_k_lock),

        .RoraLink_8B10B_Top_user_clk_i          (rx_clk),
        .RoraLink_8B10B_Top_init_clk_i          (clk),
        .RoraLink_8B10B_Top_reset_i             (!rst_n_50m),
        .RoraLink_8B10B_Top_user_pll_locked_i   (gt_pll_ok),
        .RoraLink_8B10B_Top_gt_reset_i          (1'b0),
        .RoraLink_8B10B_Top_gt_pcs_rx_reset_i   (1'b0)
    );

    // ------------------------------------------------------------------------
    // RX clock domain reset
    // ------------------------------------------------------------------------
    reg [2:0] rstn_rx_sync;
    always @(posedge rx_clk or negedge rst_n) begin
        if (!rst_n)
            rstn_rx_sync <= 3'b000;
        else
            rstn_rx_sync <= {rstn_rx_sync[1:0], 1'b1};
    end

    wire rx_rst = (!rstn_rx_sync[2]) || rl_sys_reset || link_reset || (!gt_pll_ok) || (!channel_up);

    // ------------------------------------------------------------------------
    // Expected video colorbar, must match 15K TX top.
    // ------------------------------------------------------------------------
    function [23:0] colorbar_rgb;
        input [11:0] x;
        input [10:0] y;
        begin
            if (x < 12'd240)
                colorbar_rgb = 24'hFFFFFF;
            else if (x < 12'd480)
                colorbar_rgb = 24'hFFFF00;
            else if (x < 12'd720)
                colorbar_rgb = 24'h00FFFF;
            else if (x < 12'd960)
                colorbar_rgb = 24'h00FF00;
            else if (x < 12'd1200)
                colorbar_rgb = 24'hFF00FF;
            else if (x < 12'd1440)
                colorbar_rgb = 24'hFF0000;
            else if (x < 12'd1680)
                colorbar_rgb = 24'h0000FF;
            else
                colorbar_rgb = 24'h000000;
        end
    endfunction

    function [31:0] expected_packet_word;
        input [11:0] word_idx;
        input [15:0] fid;
        input [10:0] lid;
        reg   [11:0] px;
        begin
            if (word_idx >= HEADER_WORDS)
                px = word_idx - HEADER_WORDS;
            else
                px = 12'd0;

            case (word_idx)
                12'd0: expected_packet_word = MAGIC_WORD;
                12'd1: expected_packet_word = {fid, 5'd0, lid};
                12'd2: expected_packet_word = {16'd1920, 16'd1080};
                12'd3: expected_packet_word = {8'h01, 8'h00, 16'd1920};
                default: expected_packet_word = {8'h00, colorbar_rgb(px, lid)};
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // Packet checker
    // ------------------------------------------------------------------------
    reg [11:0] word_idx;
    reg [15:0] cur_frame_id;
    reg [10:0] cur_line_id;
    reg [15:0] exp_frame_id;
    reg [10:0] exp_line_id;
    reg        exp_valid;
    reg        frame_error_cur;

    reg [31:0] rx_valid_cnt;
    reg [31:0] rx_line_cnt;
    reg [31:0] rx_crc_valid_cnt;
    reg [31:0] rx_crc_pass_cnt;
    reg [15:0] rx_good_line_cnt;

    reg        seen_valid;
    reg        seen_last;
    reg        seen_crc_valid;
    reg        seen_crc_pass;

    reg        payload_err_seen;
    reg        header_err_seen;
    reg        seq_err_seen;
    reg        last_err_seen;
    reg        crc_err_seen;
    reg        hard_err_seen;
    reg        soft_err_seen;
    reg        frame_err_seen;
    reg        strb_err_seen;
    reg        overrun_err_seen;
    reg        any_err_seen;
    reg        test_pass;

    reg [31:0] first_bad_data;
    reg [31:0] first_bad_expect;
    reg [11:0] first_bad_word_idx;
    reg [15:0] first_bad_frame_id;
    reg [10:0] first_bad_line_id;

    wire [15:0] rx_word_frame_id = user_rx_data[31:16];
    wire [10:0] rx_word_line_id  = user_rx_data[10:0];
    wire        rx_word_reserved_bad = (user_rx_data[15:11] != 5'd0);

    wire [31:0] expected_now = expected_packet_word(word_idx, cur_frame_id, cur_line_id);
    wire        payload_region = (word_idx >= HEADER_WORDS) && (word_idx < PACKET_WORDS);
    wire        last_expected  = (word_idx == PACKET_WORDS - 1);

    wire        strb_bad_now = user_rx_valid && (user_rx_strb != 4'hF);
    wire        magic_bad_now = user_rx_valid && (word_idx == 12'd0) && (user_rx_data != MAGIC_WORD);
    wire        word1_bad_now = user_rx_valid && (word_idx == 12'd1) &&
                                (rx_word_reserved_bad ||
                                 (exp_valid && ((rx_word_frame_id != exp_frame_id) || (rx_word_line_id != exp_line_id))));
    wire        word2_bad_now = user_rx_valid && (word_idx == 12'd2) && (user_rx_data != {16'd1920, 16'd1080});
    wire        word3_bad_now = user_rx_valid && (word_idx == 12'd3) && (user_rx_data != {8'h01, 8'h00, 16'd1920});
    wire        payload_bad_now = user_rx_valid && payload_region && (user_rx_data != expected_now);
    wire        overrun_bad_now = user_rx_valid && (word_idx >= PACKET_WORDS);
    wire        last_bad_now = user_rx_valid && (user_rx_last != last_expected);

    wire        header_bad_now = magic_bad_now | word1_bad_now | word2_bad_now | word3_bad_now;
    wire        word_bad_now = strb_bad_now | header_bad_now | payload_bad_now | overrun_bad_now | last_bad_now;
    wire        crc_bad_now = crc_valid && !crc_pass_fail_n;
    wire        protocol_err_now = hard_err | soft_err | frame_err | crc_bad_now | word_bad_now;

    always @(posedge rx_clk) begin
        if (rx_rst) begin
            word_idx           <= 12'd0;
            cur_frame_id       <= 16'd0;
            cur_line_id        <= 11'd0;
            exp_frame_id       <= 16'd0;
            exp_line_id        <= 11'd0;
            exp_valid          <= 1'b0;
            frame_error_cur    <= 1'b0;

            rx_valid_cnt       <= 32'd0;
            rx_line_cnt        <= 32'd0;
            rx_crc_valid_cnt   <= 32'd0;
            rx_crc_pass_cnt    <= 32'd0;
            rx_good_line_cnt   <= 16'd0;

            seen_valid         <= 1'b0;
            seen_last          <= 1'b0;
            seen_crc_valid     <= 1'b0;
            seen_crc_pass      <= 1'b0;

            payload_err_seen   <= 1'b0;
            header_err_seen    <= 1'b0;
            seq_err_seen       <= 1'b0;
            last_err_seen      <= 1'b0;
            crc_err_seen       <= 1'b0;
            hard_err_seen      <= 1'b0;
            soft_err_seen      <= 1'b0;
            frame_err_seen     <= 1'b0;
            strb_err_seen      <= 1'b0;
            overrun_err_seen   <= 1'b0;
            any_err_seen       <= 1'b0;
            test_pass          <= 1'b0;

            first_bad_data     <= 32'd0;
            first_bad_expect   <= 32'd0;
            first_bad_word_idx <= 12'd0;
            first_bad_frame_id <= 16'd0;
            first_bad_line_id  <= 11'd0;
        end else begin
            if (hard_err) begin
                hard_err_seen <= 1'b1;
                any_err_seen  <= 1'b1;
            end
            if (soft_err) begin
                soft_err_seen <= 1'b1;
                any_err_seen  <= 1'b1;
            end
            if (frame_err) begin
                frame_err_seen <= 1'b1;
                any_err_seen   <= 1'b1;
            end
            if (crc_valid) begin
                seen_crc_valid   <= 1'b1;
                rx_crc_valid_cnt <= rx_crc_valid_cnt + 32'd1;
                if (crc_pass_fail_n) begin
                    seen_crc_pass    <= 1'b1;
                    rx_crc_pass_cnt  <= rx_crc_pass_cnt + 32'd1;
                end else begin
                    crc_err_seen <= 1'b1;
                    any_err_seen <= 1'b1;
                end
            end

            if (user_rx_valid) begin
                seen_valid   <= 1'b1;
                rx_valid_cnt <= rx_valid_cnt + 32'd1;

                if (word_idx == 12'd1) begin
                    cur_frame_id <= rx_word_frame_id;
                    cur_line_id  <= rx_word_line_id;
                end

                if (word_bad_now) begin
                    frame_error_cur <= 1'b1;
                    any_err_seen    <= 1'b1;

                    if (strb_bad_now)
                        strb_err_seen <= 1'b1;
                    if (magic_bad_now | word2_bad_now | word3_bad_now)
                        header_err_seen <= 1'b1;
                    if (word1_bad_now) begin
                        header_err_seen <= 1'b1;
                        if (exp_valid && ((rx_word_frame_id != exp_frame_id) || (rx_word_line_id != exp_line_id)))
                            seq_err_seen <= 1'b1;
                    end
                    if (payload_bad_now)
                        payload_err_seen <= 1'b1;
                    if (overrun_bad_now)
                        overrun_err_seen <= 1'b1;
                    if (last_bad_now)
                        last_err_seen <= 1'b1;

                    if (!any_err_seen) begin
                        first_bad_data     <= user_rx_data;
                        first_bad_expect   <= (word_idx == 12'd1) ? {exp_frame_id, 5'd0, exp_line_id} : expected_now;
                        first_bad_word_idx <= word_idx;
                        first_bad_frame_id <= cur_frame_id;
                        first_bad_line_id  <= cur_line_id;
                    end
                end

                if (user_rx_last) begin
                    seen_last   <= 1'b1;
                    rx_line_cnt <= rx_line_cnt + 32'd1;

                    if (!frame_error_cur && !word_bad_now) begin
                        if (rx_good_line_cnt != 16'hFFFF)
                            rx_good_line_cnt <= rx_good_line_cnt + 16'd1;
                    end

                    // After a received line, expect the next line/frame.
                    exp_valid <= 1'b1;
                    if (cur_line_id == V_RES - 1) begin
                        exp_line_id  <= 11'd0;
                        exp_frame_id <= cur_frame_id + 16'd1;
                    end else begin
                        exp_line_id  <= cur_line_id + 11'd1;
                        exp_frame_id <= cur_frame_id;
                    end

                    word_idx        <= 12'd0;
                    frame_error_cur <= 1'b0;
                end else begin
                    if (word_idx != 12'hFFF)
                        word_idx <= word_idx + 12'd1;
                end
            end

            if ((rx_good_line_cnt >= PASS_LINE_TH) && seen_crc_pass && !any_err_seen)
                test_pass <= 1'b1;
            else if (any_err_seen)
                test_pass <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // LEDs in rx_clk domain
    // ------------------------------------------------------------------------
    reg [25:0] led_cnt;
    always @(posedge rx_clk or negedge rst_n) begin
        if (!rst_n)
            led_cnt <= 26'd0;
        else
            led_cnt <= led_cnt + 26'd1;
    end

    wire led_slow = led_cnt[25];
    wire led_fast = led_cnt[22];

    assign O_led[0] = gt_pll_ok & lane_up & channel_up;
    assign O_led[1] = (!gt_pll_ok)      ? 1'b0    :
                      (any_err_seen)    ? led_fast:
                      (test_pass)       ? 1'b1    :
                                          led_slow;

    // ------------------------------------------------------------------------
    // ILA-friendly debug signals. Search prefix: ila60rl_rx_
    // Recommended ILA clock: rx_clk = RoraLink_8B10B_Top_gt_pcs_rx_clk_o
    // ------------------------------------------------------------------------
    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_top_version       = TOP_VERSION;
    (* syn_keep = 1 *) wire        ila60rl_rx_rst               = rx_rst;
    (* syn_keep = 1 *) wire        ila60rl_rx_rl_sys_reset      = rl_sys_reset;
    (* syn_keep = 1 *) wire        ila60rl_rx_link_reset        = link_reset;

    (* syn_keep = 1 *) wire        ila60rl_rx_gt_pll_ok         = gt_pll_ok;
    (* syn_keep = 1 *) wire        ila60rl_rx_lane_up           = lane_up;
    (* syn_keep = 1 *) wire        ila60rl_rx_channel_up        = channel_up;
    (* syn_keep = 1 *) wire        ila60rl_rx_gt_rx_pma_lock    = gt_rx_pma_lock;
    (* syn_keep = 1 *) wire        ila60rl_rx_gt_rx_k_lock      = gt_rx_k_lock;
    (* syn_keep = 1 *) wire        ila60rl_rx_gt_rx_align_link  = gt_rx_align_link;

    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_user_rx_data      = user_rx_data;
    (* syn_keep = 1 *) wire [3:0]  ila60rl_rx_user_rx_strb      = user_rx_strb;
    (* syn_keep = 1 *) wire        ila60rl_rx_user_rx_valid     = user_rx_valid;
    (* syn_keep = 1 *) wire        ila60rl_rx_user_rx_last      = user_rx_last;
    (* syn_keep = 1 *) wire [11:0] ila60rl_rx_word_idx          = word_idx;
    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_expected_now      = expected_now;

    (* syn_keep = 1 *) wire [15:0] ila60rl_rx_cur_frame_id      = cur_frame_id;
    (* syn_keep = 1 *) wire [10:0] ila60rl_rx_cur_line_id       = cur_line_id;
    (* syn_keep = 1 *) wire [15:0] ila60rl_rx_exp_frame_id      = exp_frame_id;
    (* syn_keep = 1 *) wire [10:0] ila60rl_rx_exp_line_id       = exp_line_id;
    (* syn_keep = 1 *) wire        ila60rl_rx_exp_valid         = exp_valid;

    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_valid_cnt         = rx_valid_cnt;
    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_line_cnt          = rx_line_cnt;
    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_crc_valid_cnt     = rx_crc_valid_cnt;
    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_crc_pass_cnt      = rx_crc_pass_cnt;
    (* syn_keep = 1 *) wire [15:0] ila60rl_rx_good_line_cnt     = rx_good_line_cnt;

    (* syn_keep = 1 *) wire        ila60rl_rx_crc_valid         = crc_valid;
    (* syn_keep = 1 *) wire        ila60rl_rx_crc_pass_fail_n   = crc_pass_fail_n;
    (* syn_keep = 1 *) wire        ila60rl_rx_hard_err          = hard_err;
    (* syn_keep = 1 *) wire        ila60rl_rx_soft_err          = soft_err;
    (* syn_keep = 1 *) wire        ila60rl_rx_frame_err         = frame_err;

    (* syn_keep = 1 *) wire        ila60rl_rx_magic_bad_now     = magic_bad_now;
    (* syn_keep = 1 *) wire        ila60rl_rx_word1_bad_now     = word1_bad_now;
    (* syn_keep = 1 *) wire        ila60rl_rx_payload_bad_now   = payload_bad_now;
    (* syn_keep = 1 *) wire        ila60rl_rx_last_bad_now      = last_bad_now;
    (* syn_keep = 1 *) wire        ila60rl_rx_crc_bad_now       = crc_bad_now;
    (* syn_keep = 1 *) wire        ila60rl_rx_any_err_now       = protocol_err_now;

    (* syn_keep = 1 *) wire        ila60rl_rx_seen_valid        = seen_valid;
    (* syn_keep = 1 *) wire        ila60rl_rx_seen_last         = seen_last;
    (* syn_keep = 1 *) wire        ila60rl_rx_seen_crc_valid    = seen_crc_valid;
    (* syn_keep = 1 *) wire        ila60rl_rx_seen_crc_pass     = seen_crc_pass;

    (* syn_keep = 1 *) wire        ila60rl_rx_payload_err_seen  = payload_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_header_err_seen   = header_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_seq_err_seen      = seq_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_last_err_seen     = last_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_crc_err_seen      = crc_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_hard_err_seen     = hard_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_soft_err_seen     = soft_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_frame_err_seen    = frame_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_strb_err_seen     = strb_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_overrun_err_seen  = overrun_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_any_err_seen      = any_err_seen;
    (* syn_keep = 1 *) wire        ila60rl_rx_test_pass         = test_pass;

    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_first_bad_data    = first_bad_data;
    (* syn_keep = 1 *) wire [31:0] ila60rl_rx_first_bad_expect  = first_bad_expect;
    (* syn_keep = 1 *) wire [11:0] ila60rl_rx_first_bad_word_idx= first_bad_word_idx;
    (* syn_keep = 1 *) wire [15:0] ila60rl_rx_first_bad_frame_id= first_bad_frame_id;
    (* syn_keep = 1 *) wire [10:0] ila60rl_rx_first_bad_line_id = first_bad_line_id;

endmodule
