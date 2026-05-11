// ============================================================================
// HDMI output stage
// ----------------------------------------------------------------------------
// Registers HDMI timing and RGB data in pixel_clk domain before driving ADV7513.
// ============================================================================
`timescale 1ns / 1ps

module hdmi_output_stage (
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,

    input  wire        hs_raw,
    input  wire        vs_raw,
    input  wire        de_raw,
    input  wire [31:0] pixel_word,
    input  wire        display_started,
    input  wire        fb_error,

    output wire        adv_clk,
    output wire        adv_hs,
    output wire        adv_vs,
    output wire        adv_de,
    output wire [23:0] adv_data,

    output wire [23:0] rgb_dbg
);

reg        hs_d;
reg        vs_d;
reg        de_d;
reg [23:0] rgb_d;

wire [23:0] ddr_rgb      = pixel_word[23:0];
wire [23:0] fallback_rgb = 24'hFFFFFF;
wire        use_ddr_rgb  = display_started && !fb_error;

always @(posedge pixel_clk or negedge pixel_rst_n) begin
    if (!pixel_rst_n) begin
        hs_d  <= 1'b0;
        vs_d  <= 1'b0;
        de_d  <= 1'b0;
        rgb_d <= 24'h000000;
    end else begin
        hs_d <= hs_raw;
        vs_d <= vs_raw;
        de_d <= de_raw;
        if (de_raw)
            rgb_d <= use_ddr_rgb ? ddr_rgb : fallback_rgb;
        else
            rgb_d <= 24'h000000;
    end
end

assign adv_clk  = pixel_clk;
assign adv_hs   = hs_d;
assign adv_vs   = vs_d;
assign adv_de   = de_d;
assign adv_data = rgb_d;
assign rgb_dbg  = rgb_d;

endmodule
