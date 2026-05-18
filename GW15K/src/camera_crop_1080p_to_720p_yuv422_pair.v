// ============================================================================
// 1920x1080 YUV422-pair stream center crop to 1280x720
// ----------------------------------------------------------------------------
// Input : one valid beat carries two YUV422 pixels in one 32-bit word:
//         {Y1, V, Y0, U} in the same byte layout used by MIPI_YUV422toRGB888:
//         yuv[7:0]=U, yuv[15:8]=Y0, yuv[23:16]=V, yuv[31:24]=Y1.
// Output: one valid beat carries one cropped YUV422 pair.
//
// Default center crop:
//   X: 320  .. 1599  (1280 pixels / 640 YUV422-pair words)
//   Y: 180  .. 899   (720 lines)
// ============================================================================

`timescale 1ns / 1ps

module camera_crop_1080p_to_720p_yuv422_pair #(
    parameter [11:0] IN_H_RES      = 12'd1920,
    parameter [10:0] IN_V_RES      = 11'd1080,
    parameter [11:0] CROP_X_START  = 12'd320,
    parameter [10:0] CROP_Y_START  = 11'd180,
    parameter [11:0] OUT_H_RES     = 12'd1280,
    parameter [10:0] OUT_V_RES     = 11'd720
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_fv,
    input  wire        in_lv,
    input  wire        in_de,
    input  wire [31:0] in_yuv422_pair,

    output reg         out_valid,
    output reg  [31:0] out_yuv422_pair,
    output reg         out_sof,
    output reg         out_eol,
    output reg         out_eof,
    output reg  [10:0] out_line_id,
    output reg  [10:0] out_pair_x,

    output reg         frame_seen,
    output reg  [31:0] crop_frame_cnt,
    output reg  [15:0] crop_pair_cnt,
    output reg  [15:0] crop_line_cnt
);

localparam [11:0] CROP_X_END_PAIR = CROP_X_START + OUT_H_RES - 12'd2; // last pair x
localparam [10:0] CROP_Y_END      = CROP_Y_START + OUT_V_RES - 11'd1;

reg        fv_d;
reg        lv_d;
reg [11:0] in_x;
reg [10:0] in_y;
reg        frame_locked;

wire fv_rise =  in_fv & ~fv_d;
wire lv_rise =  in_lv & ~lv_d;
wire lv_fall = ~in_lv &  lv_d;

wire in_crop_y = (in_y >= CROP_Y_START) && (in_y <= CROP_Y_END);
wire in_crop_x = (in_x >= CROP_X_START) && (in_x <= CROP_X_END_PAIR);
wire accept_pair = frame_locked && in_fv && in_lv && in_de && in_crop_y && in_crop_x;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fv_d             <= 1'b0;
        lv_d             <= 1'b0;
        in_x             <= 12'd0;
        in_y             <= 11'd0;
        frame_locked     <= 1'b0;
        out_valid        <= 1'b0;
        out_yuv422_pair  <= 32'd0;
        out_sof          <= 1'b0;
        out_eol          <= 1'b0;
        out_eof          <= 1'b0;
        out_line_id      <= 11'd0;
        out_pair_x       <= 11'd0;
        frame_seen       <= 1'b0;
        crop_frame_cnt   <= 32'd0;
        crop_pair_cnt    <= 16'd0;
        crop_line_cnt    <= 16'd0;
    end else begin
        fv_d <= in_fv;
        lv_d <= in_lv;

        out_valid <= 1'b0;
        out_sof   <= 1'b0;
        out_eol   <= 1'b0;
        out_eof   <= 1'b0;

        if (fv_rise) begin
            in_y           <= 11'd0;
            in_x           <= 12'd0;
            frame_locked   <= 1'b1;
            frame_seen     <= 1'b1;
            crop_frame_cnt <= crop_frame_cnt + 1'b1;
            crop_pair_cnt  <= 16'd0;
            crop_line_cnt  <= 16'd0;
        end

        if (lv_rise && in_fv) begin
            in_x <= 12'd0;
        end else if (in_de && in_fv && in_lv) begin
            in_x <= in_x + 12'd2;
        end

        if (lv_fall && in_fv) begin
            in_y <= in_y + 1'b1;
        end

        if (accept_pair) begin
            out_valid       <= 1'b1;
            out_yuv422_pair <= in_yuv422_pair;
            out_line_id     <= in_y - CROP_Y_START;
            out_pair_x      <= in_x - CROP_X_START;

            out_sof <= (in_y == CROP_Y_START) && (in_x == CROP_X_START);
            out_eol <= (in_x == CROP_X_END_PAIR);
            out_eof <= (in_y == CROP_Y_END) && (in_x == CROP_X_END_PAIR);

            crop_pair_cnt <= crop_pair_cnt + 1'b1;
            if (in_x == CROP_X_START)
                crop_line_cnt <= crop_line_cnt + 1'b1;
        end
    end
end

endmodule
