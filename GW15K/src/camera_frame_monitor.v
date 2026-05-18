// ============================================================================
// Simple camera frame monitor
// ----------------------------------------------------------------------------
// Counts FV/LV/DE activity in the camera pixel domain.
// pixel_inc should be 1 for one-pixel-per-cycle streams or 2 for two-pixel
// packed streams such as RGB888 pair output from MIPI_YUV422toRGB888.
// ============================================================================

`timescale 1ns / 1ps

module camera_frame_monitor (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        fv,
    input  wire        lv,
    input  wire        de,
    input  wire [1:0]  pixel_inc,

    output reg  [31:0] frame_cnt,
    output reg  [15:0] line_cnt_cur,
    output reg  [15:0] line_cnt_last,
    output reg  [15:0] pixel_cnt_cur,
    output reg  [15:0] pixel_cnt_last,
    output reg  [15:0] pixel_cnt_max,
    output reg         frame_seen,
    output reg         line_seen,
    output reg         pixel_seen
);

reg fv_d;
reg lv_d;

wire fv_rise =  fv & ~fv_d;
wire fv_fall = ~fv &  fv_d;
wire lv_rise =  lv & ~lv_d;
wire lv_fall = ~lv &  lv_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fv_d           <= 1'b0;
        lv_d           <= 1'b0;
        frame_cnt      <= 32'd0;
        line_cnt_cur   <= 16'd0;
        line_cnt_last  <= 16'd0;
        pixel_cnt_cur  <= 16'd0;
        pixel_cnt_last <= 16'd0;
        pixel_cnt_max  <= 16'd0;
        frame_seen     <= 1'b0;
        line_seen      <= 1'b0;
        pixel_seen     <= 1'b0;
    end else begin
        fv_d <= fv;
        lv_d <= lv;

        if (fv_rise) begin
            frame_cnt     <= frame_cnt + 1'b1;
            line_cnt_cur  <= 16'd0;
            pixel_cnt_cur <= 16'd0;
            frame_seen    <= 1'b1;
        end

        if (lv_fall) begin
            pixel_cnt_last <= pixel_cnt_cur;
            if (pixel_cnt_cur > pixel_cnt_max)
                pixel_cnt_max <= pixel_cnt_cur;
        end

        if (fv_fall) begin
            line_cnt_last <= line_cnt_cur;
        end

        // Important: line-start reset must have priority over normal DE count.
        // If LV rises on the same cycle as DE, count the first valid pair as
        // pixel_inc instead of allowing pixel_cnt_cur to continue from last line.
        if (lv_rise && fv) begin
            line_cnt_cur <= line_cnt_cur + 1'b1;
            line_seen    <= 1'b1;
            if (de)
                pixel_cnt_cur <= {14'd0, pixel_inc};
            else
                pixel_cnt_cur <= 16'd0;
        end else if (de && fv && lv) begin
            pixel_cnt_cur <= pixel_cnt_cur + pixel_inc;
            pixel_seen    <= 1'b1;
        end
    end
end

endmodule
