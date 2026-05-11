// ============================================================================
// ADV7513 I2C initialization wrapper
// ----------------------------------------------------------------------------
// Copies the verified GW60K_cb-style ADV7513 initialization timing and keeps
// the I2C init/master instances out of top.v.
// ============================================================================
`timescale 1ns / 1ps

module adv7513_iic_wrapper (
    input  wire clk,       // 50MHz board clock
    input  wire rst_n_50m,
    inout  wire scl,
    inout  wire sda
);

localparam [27:0] IIC_RESET_DELAY_CNT = 28'd50_000_000;
localparam [27:0] IIC_START_DELAY_CNT = 28'd100_000_000;

reg [27:0] iic_delay_cnt;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        iic_delay_cnt <= 28'd0;
    else if (iic_delay_cnt < IIC_START_DELAY_CNT)
        iic_delay_cnt <= iic_delay_cnt + 1'b1;
    else
        iic_delay_cnt <= iic_delay_cnt;
end

wire adv_iic_reset_n = rst_n_50m && (iic_delay_cnt >= IIC_RESET_DELAY_CNT);
wire adv_iic_start   = rst_n_50m && (iic_delay_cnt >= IIC_START_DELAY_CNT);

wire       TX_EN_7513;
wire [2:0] WADDR_7513;
wire [7:0] WDATA_7513;
wire       RX_EN_7513;
wire [2:0] RADDR_7513;
wire [7:0] RDATA_7513;

adv7513_iic_init u_adv7513_iic_init (
    .I_CLK       (clk),
    .I_RESETN    (adv_iic_reset_n),
    .start       (adv_iic_start),
    .O_TX_EN     (TX_EN_7513),
    .O_WADDR     (WADDR_7513),
    .O_WDATA     (WDATA_7513),
    .O_RX_EN     (RX_EN_7513),
    .O_RADDR     (RADDR_7513),
    .I_RDATA     (RDATA_7513),
    .cstate_flag (),
    .error_flag  ()
);

I2C_MASTER_Top u_i2c_master (
    .I_CLK     (clk),
    .I_RESETN  (adv_iic_reset_n),
    .I_TX_EN   (TX_EN_7513),
    .I_WADDR   (WADDR_7513),
    .I_WDATA   (WDATA_7513),
    .I_RX_EN   (RX_EN_7513),
    .I_RADDR   (RADDR_7513),
    .O_RDATA   (RDATA_7513),
    .O_IIC_INT (),
    .SCL       (scl),
    .SDA       (sda)
);

endmodule
