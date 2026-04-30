module top
(
    input             clk                 ,  // 50MHz板上晶振
    input             rst_n               ,

    output     [1:0]  O_led               ,

    inout             IO_adv7513_scl      ,
    inout             IO_adv7513_sda      ,
    output            O_adv7513_clk       ,
    output            O_adv7513_vs        ,
    output            O_adv7513_hs        ,
    output            O_adv7513_de        ,
    output     [23:0] O_adv7513_data
);


// ================================================================
// 1080P60 timing
// Pixel clock: 148.5MHz
// ================================================================
localparam [15:0] H_TOTAL  = 16'd2200;
localparam [15:0] H_SYNC   = 16'd44;
localparam [15:0] H_BPORCH = 16'd148;
localparam [15:0] H_RES    = 16'd1920;

localparam [15:0] V_TOTAL  = 16'd1125;
localparam [15:0] V_SYNC   = 16'd5;
localparam [15:0] V_BPORCH = 16'd36;
localparam [15:0] V_RES    = 16'd1080;


// ================================================================
// Clock / PLL
// ================================================================
wire pixel_clk;
wire pixel_pll_lock;

Gowin_PLL u_pixel_pll
(
    .clkin   (clk            ), // input  clkin, 50MHz
    .clkout0 (pixel_clk      ), // output clkout0, 148.5MHz
    .lock    (pixel_pll_lock ), // output lock
    .mdclk   (clk            ), // input  mdclk
    .reset   (!rst_n         )  // input  reset, high active
);


// ================================================================
// Reset sync
// ================================================================
// 总复位：外部复位释放，并且 PLL locked 后，系统才开始工作
wire global_rst_n = rst_n & pixel_pll_lock;


// -------------------------------
// 50MHz system clock reset
// -------------------------------
reg [2:0] sys_rst_sync;

always @(posedge clk or negedge global_rst_n)
begin
    if (!global_rst_n)
        sys_rst_sync <= 3'b000;
    else
        sys_rst_sync <= {sys_rst_sync[1:0], 1'b1};
end

wire sys_rst_n = sys_rst_sync[2];


// -------------------------------
// 148.5MHz pixel clock reset
// -------------------------------
reg [2:0] pixel_rst_sync;

always @(posedge pixel_clk or negedge global_rst_n)
begin
    if (!global_rst_n)
        pixel_rst_sync <= 3'b000;
    else
        pixel_rst_sync <= {pixel_rst_sync[1:0], 1'b1};
end

wire pixel_rst_n = pixel_rst_sync[2];


// ================================================================
// ADV7513 I2C delayed reset/start
// 50MHz:
//   50_000_000  = 1s
//   100_000_000 = 2s
// ================================================================
localparam [27:0] IIC_RESET_DELAY_CNT = 28'd50_000_000;
localparam [27:0] IIC_START_DELAY_CNT = 28'd100_000_000;

reg [27:0] iic_delay_cnt;

always @(posedge clk or negedge sys_rst_n)
begin
    if (!sys_rst_n)
        iic_delay_cnt <= 28'd0;
    else if (iic_delay_cnt < IIC_START_DELAY_CNT)
        iic_delay_cnt <= iic_delay_cnt + 1'b1;
    else
        iic_delay_cnt <= iic_delay_cnt;
end

wire adv_iic_reset_n = sys_rst_n && (iic_delay_cnt >= IIC_RESET_DELAY_CNT);
wire adv_iic_start   = sys_rst_n && (iic_delay_cnt >= IIC_START_DELAY_CNT);


// ================================================================
// LED
// O_led[0]: 148.5MHz pixel clock heartbeat
// O_led[1]: 50MHz system clock heartbeat
// ================================================================
localparam [31:0] PIXEL_LED_PERIOD = 32'd148_500_000;
localparam [31:0] SYS_LED_PERIOD   = 32'd50_000_000;

reg [31:0] run_cnt0;
reg [31:0] run_cnt1;

wire running0;
wire running1;

always @(posedge pixel_clk or negedge pixel_rst_n)
begin
    if (!pixel_rst_n)
        run_cnt0 <= 32'd0;
    else if (run_cnt0 >= PIXEL_LED_PERIOD - 1'b1)
        run_cnt0 <= 32'd0;
    else
        run_cnt0 <= run_cnt0 + 1'b1;
end

assign running0 = (run_cnt0 < (PIXEL_LED_PERIOD >> 1)) ? 1'b1 : 1'b0;


always @(posedge clk or negedge sys_rst_n)
begin
    if (!sys_rst_n)
        run_cnt1 <= 32'd0;
    else if (run_cnt1 >= SYS_LED_PERIOD - 1'b1)
        run_cnt1 <= 32'd0;
    else
        run_cnt1 <= run_cnt1 + 1'b1;
end

assign running1 = (run_cnt1 < (SYS_LED_PERIOD >> 1)) ? 1'b1 : 1'b0;

assign O_led[0] = running0;
assign O_led[1] = running1;


// ================================================================
// ADV7513 I2C wires
// ================================================================
wire         TX_EN_7513;
wire [2:0]   WADDR_7513;
wire [7:0]   WDATA_7513;
wire         RX_EN_7513;
wire [2:0]   RADDR_7513;
wire [7:0]   RDATA_7513;


// ================================================================
// ADV7513 I2C init
// I_CLK 使用板载 50MHz clk
// ================================================================
adv7513_iic_init adv7513_iic_init_inst0
(
    .I_CLK        (clk              ),
    .I_RESETN     (adv_iic_reset_n  ),
    .start        (adv_iic_start    ),

    .O_TX_EN      (TX_EN_7513       ),
    .O_WADDR      (WADDR_7513       ),
    .O_WDATA      (WDATA_7513       ),

    .O_RX_EN      (RX_EN_7513       ),
    .O_RADDR      (RADDR_7513       ),
    .I_RDATA      (RDATA_7513       ),

    .cstate_flag  (                 ),
    .error_flag   (                 )
);


I2C_MASTER_Top I2C_MASTER_Top_inst0
(
    .I_CLK      (clk              ),
    .I_RESETN   (adv_iic_reset_n  ),

    .I_TX_EN    (TX_EN_7513       ),
    .I_WADDR    (WADDR_7513       ),
    .I_WDATA    (WDATA_7513       ),

    .I_RX_EN    (RX_EN_7513       ),
    .I_RADDR    (RADDR_7513       ),
    .O_RDATA    (RDATA_7513       ),

    .O_IIC_INT  (                 ),
    .SCL        (IO_adv7513_scl   ),
    .SDA        (IO_adv7513_sda   )
);


// ================================================================
// Testpattern output wires
// ================================================================
wire [15:0] tp_vs_cnt;
wire [15:0] tp_hs_cnt;

wire        tp_vs;
wire        tp_hs;
wire        tp_de;

wire [7:0]  tp_r;
wire [7:0]  tp_g;
wire [7:0]  tp_b;


// ================================================================
// Testpattern
// Direct RGB color bar source for HDMI
// ================================================================
testpattern testpattern_inst0
(
    .I_pxl_clk    (pixel_clk    ),
    .I_rst_n      (pixel_rst_n  ),

    .I_mode       (3'd0         ), // color bar mode，如果不是彩条模式，就改这里
    .I_sqr_width  (16'd60       ),

    .I_single_r   (8'd0         ),
    .I_single_g   (8'd255       ),
    .I_single_b   (8'd0         ),

    .I_h_total    (H_TOTAL      ),
    .I_h_sync     (H_SYNC       ),
    .I_h_bporch   (H_BPORCH     ),
    .I_h_res      (H_RES        ),

    .I_v_total    (V_TOTAL      ),
    .I_v_sync     (V_SYNC       ),
    .I_v_bporch   (V_BPORCH     ),
    .I_v_res      (V_RES        ),

    .I_hs_pol     (1'b1         ),
    .I_vs_pol     (1'b1         ),

    .O_V_cnt      (tp_vs_cnt    ),
    .O_H_cnt      (tp_hs_cnt    ),

    .O_de         (tp_de        ),
    .O_hs         (tp_hs        ),
    .O_vs         (tp_vs        ),

    .O_data_r     (tp_r         ),
    .O_data_g     (tp_g         ),
    .O_data_b     (tp_b         )
);


// ================================================================
// HDMI ADV7513 output
// testpattern RGB -> ADV7513 directly
// ================================================================
assign O_adv7513_data = {tp_r, tp_g, tp_b};
assign O_adv7513_vs   = tp_vs;
assign O_adv7513_hs   = tp_hs;
assign O_adv7513_de   = tp_de;
assign O_adv7513_clk  = pixel_clk;


endmodule