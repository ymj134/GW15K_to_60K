// ============================================================================
// RGB888 vertical colorbar generator
// ----------------------------------------------------------------------------
// Default is 1280-wide 8-bar colorbar, 160 pixels per bar.
// ============================================================================

`timescale 1ns / 1ps

module video_colorbar_rgb #(
    parameter [11:0] BAR_WIDTH = 12'd160
)(
    input  wire [11:0] x,
    input  wire [10:0] y,
    output reg  [23:0] rgb
);

always @* begin
    // y is reserved for future patterns.
    if (x < BAR_WIDTH)
        rgb = 24'hFF_FF_FF; // white
    else if (x < (BAR_WIDTH * 2))
        rgb = 24'hFF_FF_00; // yellow
    else if (x < (BAR_WIDTH * 3))
        rgb = 24'h00_FF_FF; // cyan
    else if (x < (BAR_WIDTH * 4))
        rgb = 24'h00_FF_00; // green
    else if (x < (BAR_WIDTH * 5))
        rgb = 24'hFF_00_FF; // magenta
    else if (x < (BAR_WIDTH * 6))
        rgb = 24'hFF_00_00; // red
    else if (x < (BAR_WIDTH * 7))
        rgb = 24'h00_00_FF; // blue
    else
        rgb = 24'h00_00_00; // black
end

endmodule
