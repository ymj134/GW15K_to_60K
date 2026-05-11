// ============================================================================
// 60K clock/reset block
// ----------------------------------------------------------------------------
// Keeps board reset synchronization, HDMI pixel PLL, and DDR PLL/MDPR glue
// outside of top.v. Logic is copied from the previously verified monolithic top.
// ============================================================================
`timescale 1ns / 1ps

module clock_reset_60k (
    input  wire clk,       // 50MHz board clock
    input  wire rst_n,
    input  wire pll_stop,

    output wire rst_n_50m,
    output wire global_rst,

    output wire pixel_clk,
    output wire pixel_pll_lock,
    output wire pixel_rst_n,

    output wire memory_clk,
    output wire ddr_pll_lock
);

// --------------------------------------------------------------------------
// 50MHz reset sync
// --------------------------------------------------------------------------
reg [3:0] rst_sync;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rst_sync <= 4'b0000;
    else
        rst_sync <= {rst_sync[2:0], 1'b1};
end

assign rst_n_50m = rst_sync[3];
assign global_rst = ~rst_n_50m;

// --------------------------------------------------------------------------
// HDMI pixel PLL: 50MHz -> 74.25MHz
// --------------------------------------------------------------------------
Gowin_PLL u_pixel_pll (
    .clkin  (clk),
    .clkout0(pixel_clk),
    .lock   (pixel_pll_lock),
    .mdclk  (clk),
    .reset  (!rst_n_50m)
);

reg [3:0] pixel_rst_sync;
always @(posedge pixel_clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        pixel_rst_sync <= 4'b0000;
    else if (!pixel_pll_lock)
        pixel_rst_sync <= 4'b0000;
    else
        pixel_rst_sync <= {pixel_rst_sync[2:0], 1'b1};
end

assign pixel_rst_n = pixel_rst_sync[3];

// --------------------------------------------------------------------------
// DDR PLL + pll_stop handling
// --------------------------------------------------------------------------
wire        ddr_pll_lock_raw;
wire [7:0]  mdrp_rdata;
wire [1:0]  mdrp_op;
wire        mdrp_inc;
wire [7:0]  mdrp_wdata;

reg [15:0] pll_lock_shift;
reg        pll_stop_d;
reg        pll_mdrp_wr;

always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        pll_lock_shift <= 16'd0;
    else
        pll_lock_shift <= {pll_lock_shift[14:0], ddr_pll_lock_raw};
end
assign ddr_pll_lock = pll_lock_shift[15];

always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m) begin
        pll_stop_d  <= 1'b0;
        pll_mdrp_wr <= 1'b0;
    end else begin
        pll_stop_d  <= pll_stop;
        pll_mdrp_wr <= ddr_pll_lock && (pll_stop ^ pll_stop_d);
    end
end

Gowin_PLL_DDR u_ddr_pll (
    .lock            (ddr_pll_lock_raw),
    .clkout0         (),
    .clkout2         (memory_clk),
    .mdrdo           (mdrp_rdata),
    .clkin           (clk),
    .pll_init_bypass (ddr_pll_lock),
    .reset           (!rst_n_50m),
    .mdclk           (clk),
    .mdopc           (mdrp_op),
    .mdainc          (mdrp_inc),
    .mdwdi           (mdrp_wdata)
);

pll_mDRP_intf u_pll_mDRP_intf (
    .clk        (clk),
    .rst_n      (rst_n_50m),
    .pll_lock   (ddr_pll_lock),
    .wr         (pll_mdrp_wr),
    .mdrp_inc   (mdrp_inc),
    .mdrp_op    (mdrp_op),
    .mdrp_wdata (mdrp_wdata),
    .mdrp_rdata (mdrp_rdata)
);

endmodule
