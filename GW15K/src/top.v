`timescale 1ns / 1ps
// ============================================================================
// 15K RoraLink 8B10B TX-only video-line packet generator
// ----------------------------------------------------------------------------
// Target:
//   GW5AT-LV15MG132C1/I0
//   RoraLink 8B10B TX-only Simplex, Framing, Timer BackChannel
//   1 lane, 32-bit user data, 6.25Gbps line rate, 125MHz refclk
//
// Packet format, one RoraLink frame per video line:
//   word0 : 32'hA55A_6001
//   word1 : {frame_id[15:0], 5'd0, line_id[10:0]}
//   word2 : {16'd1920, 16'd1080}
//   word3 : {8'h01, 8'h00, 16'd1920}   // RGB888, flags, payload words
//   word4~word1923 : {8'h00, rgb888[23:0]}
// ============================================================================

module top (
    input  wire clk,      // board clock, expected 50MHz
    input  wire rst_n,    // active-low reset
    output wire led
);

    localparam [31:0] TOP_VERSION = 32'h1506_0001;

    localparam integer H_RES          = 1920;
    localparam integer V_RES          = 1080;
    localparam integer HEADER_WORDS   = 4;
    localparam integer PAYLOAD_WORDS  = H_RES;
    localparam integer PACKET_WORDS   = HEADER_WORDS + PAYLOAD_WORDS;

    // RoraLink user clock at 6.25Gbps and 4 bytes/lane is expected to be 156.25MHz.
    // 156.25MHz / 60 / 1080 ~= 2412 cycles/line.
    localparam integer LINE_PERIOD_CYCLES = 2412;
    localparam integer LINE_GAP_CYCLES    = LINE_PERIOD_CYCLES - PACKET_WORDS;

    localparam [1:0] ST_WAIT = 2'd0;
    localparam [1:0] ST_SEND = 2'd1;
    localparam [1:0] ST_GAP  = 2'd2;

    wire        rl_sys_reset;
    wire        user_tx_ready;
    wire        hard_err;
    wire        channel_up;
    wire        lane_up;
    wire        tx_clk;
    wire        gt_pll_ok;

    reg  [31:0] user_tx_data;
    wire [3:0]  user_tx_strb;
    reg         user_tx_valid;
    reg         user_tx_last;

    assign user_tx_strb = 4'hF;

    SerDes_Top u_serdes_top (
        .RoraLink_8B10B_Top_sys_reset_o          (rl_sys_reset),
        .RoraLink_8B10B_Top_user_tx_ready_o      (user_tx_ready),
        .RoraLink_8B10B_Top_hard_err_o           (hard_err),
        .RoraLink_8B10B_Top_channel_up_o         (channel_up),
        .RoraLink_8B10B_Top_lane_up_o            (lane_up),
        .RoraLink_8B10B_Top_gt_pcs_tx_clk_o      (tx_clk),
        .RoraLink_8B10B_Top_gt_pll_lock_o        (gt_pll_ok),

        .RoraLink_8B10B_Top_user_clk_i           (tx_clk),
        .RoraLink_8B10B_Top_init_clk_i           (clk),
        .RoraLink_8B10B_Top_reset_i              (~rst_n),
        .RoraLink_8B10B_Top_user_pll_locked_i    (gt_pll_ok),

        .RoraLink_8B10B_Top_user_tx_data_i       (user_tx_data),
        .RoraLink_8B10B_Top_user_tx_strb_i       (user_tx_strb),
        .RoraLink_8B10B_Top_user_tx_valid_i      (user_tx_valid),
        .RoraLink_8B10B_Top_user_tx_last_i       (user_tx_last),

        .RoraLink_8B10B_Top_gt_reset_i           (1'b0),
        .RoraLink_8B10B_Top_gt_pcs_tx_reset_i    (1'b0)
    );

    // Reset synchronization into tx_clk domain
    reg [2:0] rstn_tx_sync;
    always @(posedge tx_clk or negedge rst_n) begin
        if (!rst_n)
            rstn_tx_sync <= 3'b000;
        else
            rstn_tx_sync <= {rstn_tx_sync[1:0], 1'b1};
    end

    wire tx_rst = (!rstn_tx_sync[2]) || rl_sys_reset || (!gt_pll_ok);

    reg channel_up_d;
    reg lane_up_d;
    reg tx_fire_seen;
    reg tx_last_seen;
    reg tx_ready_seen;
    reg hard_err_seen;
    reg channel_up_seen;

    wire tx_fire = user_tx_valid && user_tx_ready;

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            channel_up_d    <= 1'b0;
            lane_up_d       <= 1'b0;
            tx_fire_seen    <= 1'b0;
            tx_last_seen    <= 1'b0;
            tx_ready_seen   <= 1'b0;
            hard_err_seen   <= 1'b0;
            channel_up_seen <= 1'b0;
        end else begin
            channel_up_d <= channel_up;
            lane_up_d    <= lane_up;

            if (tx_fire)
                tx_fire_seen <= 1'b1;
            if (tx_fire && user_tx_last)
                tx_last_seen <= 1'b1;
            if (user_tx_ready)
                tx_ready_seen <= 1'b1;
            if (hard_err)
                hard_err_seen <= 1'b1;
            if (channel_up)
                channel_up_seen <= 1'b1;
        end
    end

    // Start delay after reset/link-up
    reg [15:0] start_delay_cnt;
    reg        start_done;

    always @(posedge tx_clk) begin
        if (tx_rst || !channel_up_d) begin
            start_delay_cnt <= 16'd0;
            start_done      <= 1'b0;
        end else if (!start_done) begin
            start_delay_cnt <= start_delay_cnt + 16'd1;
            if (start_delay_cnt == 16'hFFFF)
                start_done <= 1'b1;
        end
    end

    reg [1:0]  tx_state;
    reg [11:0] pkt_word_idx;
    reg [11:0] pixel_x;
    reg [10:0] line_id;
    reg [15:0] frame_id;
    reg [15:0] gap_cnt;

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

    function [31:0] packet_word;
        input [11:0] word_idx;
        input [15:0] fid;
        input [10:0] lid;
        input [11:0] px;
        begin
            case (word_idx)
                12'd0: packet_word = 32'hA55A_6001;
                12'd1: packet_word = {fid, 5'd0, lid};
                12'd2: packet_word = {16'd1920, 16'd1080};
                12'd3: packet_word = {8'h01, 8'h00, 16'd1920};
                default: packet_word = {8'h00, colorbar_rgb(px, lid)};
            endcase
        end
    endfunction

    always @(posedge tx_clk) begin
        if (tx_rst || !channel_up_d) begin
            tx_state      <= ST_WAIT;
            user_tx_valid <= 1'b0;
            user_tx_last  <= 1'b0;
            user_tx_data  <= 32'd0;
            pkt_word_idx  <= 12'd0;
            pixel_x       <= 12'd0;
            line_id       <= 11'd0;
            frame_id      <= 16'd0;
            gap_cnt       <= 16'd0;
        end else begin
            user_tx_valid <= 1'b0;
            user_tx_last  <= 1'b0;
            user_tx_data  <= packet_word(pkt_word_idx, frame_id, line_id, pixel_x);

            case (tx_state)
                ST_WAIT: begin
                    pkt_word_idx <= 12'd0;
                    pixel_x      <= 12'd0;
                    gap_cnt      <= 16'd0;
                    if (start_done && user_tx_ready)
                        tx_state <= ST_SEND;
                end

                ST_SEND: begin
                    user_tx_valid <= 1'b1;
                    user_tx_last  <= (pkt_word_idx == PACKET_WORDS - 1);
                    user_tx_data  <= packet_word(pkt_word_idx, frame_id, line_id, pixel_x);

                    if (tx_fire) begin
                        if (pkt_word_idx == PACKET_WORDS - 1) begin
                            pkt_word_idx <= 12'd0;
                            pixel_x      <= 12'd0;
                            gap_cnt      <= 16'd0;
                            tx_state     <= ST_GAP;

                            if (line_id == V_RES - 1) begin
                                line_id  <= 11'd0;
                                frame_id <= frame_id + 16'd1;
                            end else begin
                                line_id <= line_id + 11'd1;
                            end
                        end else begin
                            pkt_word_idx <= pkt_word_idx + 12'd1;
                            if (pkt_word_idx >= HEADER_WORDS)
                                pixel_x <= pixel_x + 12'd1;
                        end
                    end
                end

                ST_GAP: begin
                    user_tx_valid <= 1'b0;
                    user_tx_last  <= 1'b0;
                    if (LINE_GAP_CYCLES <= 1) begin
                        tx_state <= ST_SEND;
                    end else if (gap_cnt == LINE_GAP_CYCLES - 1) begin
                        gap_cnt  <= 16'd0;
                        tx_state <= ST_SEND;
                    end else begin
                        gap_cnt <= gap_cnt + 16'd1;
                    end
                end

                default: begin
                    tx_state <= ST_WAIT;
                end
            endcase
        end
    end

    // LED: off=no PLL, fast blink=hard_err, on=link up and packets fired, slow blink=waiting.
    reg [25:0] led_cnt;
    always @(posedge tx_clk or negedge rst_n) begin
        if (!rst_n)
            led_cnt <= 26'd0;
        else
            led_cnt <= led_cnt + 26'd1;
    end

    wire led_slow = led_cnt[25];
    wire led_fast = led_cnt[22];

    assign led = (!gt_pll_ok) ? 1'b0 :
                 (hard_err_seen) ? led_fast :
                 (channel_up_seen && tx_fire_seen && tx_last_seen) ? 1'b1 :
                 led_slow;

    // ILA-friendly debug signals. Search prefix: ila15rl_tx_
    (* syn_keep = 1 *) wire [31:0] ila15rl_tx_top_version      = TOP_VERSION;
    (* syn_keep = 1 *) wire [1:0]  ila15rl_tx_state            = tx_state;
    (* syn_keep = 1 *) wire [15:0] ila15rl_tx_start_delay_cnt  = start_delay_cnt;
    (* syn_keep = 1 *) wire        ila15rl_tx_start_done       = start_done;

    (* syn_keep = 1 *) wire        ila15rl_tx_gt_pll_ok        = gt_pll_ok;
    (* syn_keep = 1 *) wire        ila15rl_tx_rl_sys_reset     = rl_sys_reset;
    (* syn_keep = 1 *) wire        ila15rl_tx_tx_rst           = tx_rst;
    (* syn_keep = 1 *) wire        ila15rl_tx_lane_up          = lane_up;
    (* syn_keep = 1 *) wire        ila15rl_tx_channel_up       = channel_up;
    (* syn_keep = 1 *) wire        ila15rl_tx_user_tx_ready    = user_tx_ready;
    (* syn_keep = 1 *) wire        ila15rl_tx_hard_err         = hard_err;

    (* syn_keep = 1 *) wire [31:0] ila15rl_tx_user_tx_data     = user_tx_data;
    (* syn_keep = 1 *) wire [3:0]  ila15rl_tx_user_tx_strb     = user_tx_strb;
    (* syn_keep = 1 *) wire        ila15rl_tx_user_tx_valid    = user_tx_valid;
    (* syn_keep = 1 *) wire        ila15rl_tx_user_tx_last     = user_tx_last;
    (* syn_keep = 1 *) wire        ila15rl_tx_fire             = tx_fire;

    (* syn_keep = 1 *) wire [11:0] ila15rl_tx_pkt_word_idx     = pkt_word_idx;
    (* syn_keep = 1 *) wire [11:0] ila15rl_tx_pixel_x          = pixel_x;
    (* syn_keep = 1 *) wire [10:0] ila15rl_tx_line_id          = line_id;
    (* syn_keep = 1 *) wire [15:0] ila15rl_tx_frame_id         = frame_id;
    (* syn_keep = 1 *) wire [15:0] ila15rl_tx_gap_cnt          = gap_cnt;

    (* syn_keep = 1 *) wire        ila15rl_tx_fire_seen        = tx_fire_seen;
    (* syn_keep = 1 *) wire        ila15rl_tx_last_seen        = tx_last_seen;
    (* syn_keep = 1 *) wire        ila15rl_tx_ready_seen       = tx_ready_seen;
    (* syn_keep = 1 *) wire        ila15rl_tx_channel_up_seen  = channel_up_seen;
    (* syn_keep = 1 *) wire        ila15rl_tx_hard_err_seen    = hard_err_seen;

endmodule
