// ============================================================================
// GW15K B3-0 Camera Bring-up Top + ILA
// ----------------------------------------------------------------------------
// Purpose:
//   Bring up the ISP camera MIPI input chain on 15K before connecting it to
//   the RoraLink video packetizer.
//
// Data path:
//   Camera I2C config
//      -> Gowin_MIPI_DPHY RX 4-lane
//      -> MIPI_DSI_CSI2_RX_Top
//      -> MIPI_Byte_to_Pixel_Converter_Top
//      -> MIPI_YUV422toRGB888
//      -> camera_frame_monitor + ILA signals
//
// Notes:
//   1) This top does NOT transmit RoraLink video.
//   2) Add camera/MIPI/I2C pin constraints in fpga_project.cst.
//   3) The camera module used by the reference design is assumed to need only
//      the I2C stream-on sequence already provided by I2C_ISPCAMERA_4Lanes_Config.
// ============================================================================

`timescale 1ns / 1ps

module top (
    input  wire clk,      // 50MHz board clock
    input  wire rst_n,
    output wire led,

    // ISP camera I2C
    output wire mipi_scl_io,
    inout  wire mipi_sda_io,

    // MIPI CSI-2 4-lane input
    inout  wire ck_n,
    inout  wire ck_p,
    inout  wire d0_n,
    inout  wire d0_p,
    inout  wire d1_n,
    inout  wire d1_p,
    inout  wire d2_n,
    inout  wire d2_p,
    inout  wire d3_n,
    inout  wire d3_p
);

localparam integer SYS_CLK_FREQ = 50_000_000;
localparam integer I2C_FREQ     = 250_000;

// --------------------------------------------------------------------------
// 50MHz reset sync
// --------------------------------------------------------------------------
reg [7:0] rst_sync = 8'd0;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rst_sync <= 8'd0;
    else
        rst_sync <= {rst_sync[6:0], 1'b1};
end

wire rst_n_50m = rst_sync[7];

// --------------------------------------------------------------------------
// Camera I2C init
// --------------------------------------------------------------------------
wire       i2c_sclk;
wire       i2c_sdat;
wire [8:0] i2c_config_index;
wire [7:0] i2c_lut_size;
wire [23:0] i2c_lut_data;
wire       i2c_config_done;

i2c_timing_ctrl_reg16_dat8_wronly #(
    .CLK_FREQ (SYS_CLK_FREQ),
    .I2C_FREQ (I2C_FREQ)
) u_i2c_timing_ctrl (
    .clk              (clk),
    .rst_n            (rst_n_50m),
    .i2c_sclk         (i2c_sclk),
    .i2c_sdat         (i2c_sdat),
    .i2c_config_size  ({1'b0, i2c_lut_size}),
    .i2c_config_index (i2c_config_index),
    .i2c_config_data  ({8'h34, i2c_lut_data}), // ref design uses 8'h34 slave addr
    .i2c_config_done  (i2c_config_done)
);

I2C_ISPCAMERA_4Lanes_Config u_i2c_camera_lut (
    .LUT_INDEX (i2c_config_index[7:0]),
    .LUT_DATA  (i2c_lut_data),
    .LUT_SIZE  (i2c_lut_size)
);

assign mipi_scl_io = i2c_sclk;
assign mipi_sda_io = i2c_sdat;

// --------------------------------------------------------------------------
// MIPI D-PHY RX 4-lane
// --------------------------------------------------------------------------
wire        byte_clk;
wire [7:0]  d0ln_hsrxd;
wire [7:0]  d1ln_hsrxd;
wire [7:0]  d2ln_hsrxd;
wire [7:0]  d3ln_hsrxd;
wire        d0ln_hsrxd_vld;
wire        d1ln_hsrxd_vld;
wire        d2ln_hsrxd_vld;
wire        d3ln_hsrxd_vld;

wire        di_lprx0_n;
wire        di_lprx0_p;
wire        di_lprx1_n;
wire        di_lprx1_p;
wire        di_lprx2_n;
wire        di_lprx2_p;
wire        di_lprx3_n;
wire        di_lprx3_p;
wire        di_lprxck_n;
wire        di_lprxck_p;

wire [1:0] lp_data0 = {di_lprx0_p, di_lprx0_n};
wire [1:0] lp_data1 = {di_lprx1_p, di_lprx1_n};
wire [1:0] lp_data2 = {di_lprx2_p, di_lprx2_n};
wire [1:0] lp_data3 = {di_lprx3_p, di_lprx3_n};
wire [1:0] lp_clk   = {di_lprxck_p, di_lprxck_n};

reg        odt_en_msk = 1'b0;
reg        r_3to1     = 1'b0;
reg        rx_drst_n  = 1'b1;
reg [1:0]  r_lp0_0    = 2'd0;
reg [1:0]  r_lp0_1    = 2'd0;

wire from1to0 = (r_lp0_1 == 2'd1) && (r_lp0_0 == 2'd0);
wire from1to2 = (r_lp0_1 == 2'd1) && (r_lp0_0 == 2'd2);
wire from1to3 = (r_lp0_1 == 2'd1) && (r_lp0_0 == 2'd3);
wire from3to1 = (r_lp0_1 == 2'd3) && (r_lp0_0 == 2'd1);
wire fromXto3 = (r_lp0_1 != 2'd3) && (r_lp0_0 == 2'd3);
wire from1toX = (r_lp0_1 == 2'd1) && (r_lp0_0 != 2'd1);

wire [3:0] odt_en = {
    (lp_data3 == 2'b00),
    (lp_data2 == 2'b00),
    (lp_data1 == 2'b00),
    (lp_data0 == 2'b00)
} & {4{odt_en_msk}};

always @(posedge byte_clk or negedge rst_n_50m) begin
    if (!rst_n_50m) begin
        r_3to1     <= 1'b0;
        odt_en_msk <= 1'b0;
    end else begin
        if (!r_3to1)
            r_3to1 <= from3to1;
        else
            r_3to1 <= ~from1toX;

        if (!odt_en_msk)
            odt_en_msk <= from3to1;
        else
            odt_en_msk <= !(from1to2 | from1to3 | fromXto3);
    end
end

Gowin_MIPI_DPHY u_mipi_dphy_rx_4lane (
    .rx_clk_o        (byte_clk),
    .d0ln_hsrxd      (d0ln_hsrxd),
    .d1ln_hsrxd      (d1ln_hsrxd),
    .d2ln_hsrxd      (d2ln_hsrxd),
    .d3ln_hsrxd      (d3ln_hsrxd),
    .d0ln_hsrxd_vld  (d0ln_hsrxd_vld),
    .d1ln_hsrxd_vld  (d1ln_hsrxd_vld),
    .d2ln_hsrxd_vld  (d2ln_hsrxd_vld),
    .d3ln_hsrxd_vld  (d3ln_hsrxd_vld),

    .di_lprx0_n      (di_lprx0_n),
    .di_lprx0_p      (di_lprx0_p),
    .di_lprx1_n      (di_lprx1_n),
    .di_lprx1_p      (di_lprx1_p),
    .di_lprx2_n      (di_lprx2_n),
    .di_lprx2_p      (di_lprx2_p),
    .di_lprx3_n      (di_lprx3_n),
    .di_lprx3_p      (di_lprx3_p),
    .di_lprxck_n     (di_lprxck_n),
    .di_lprxck_p     (di_lprxck_p),

    .ck_n            (ck_n),
    .ck_p            (ck_p),
    .d0_n            (d0_n),
    .d0_p            (d0_p),
    .d1_n            (d1_n),
    .d1_p            (d1_p),
    .d2_n            (d2_n),
    .d2_p            (d2_p),
    .d3_n            (d3_n),
    .d3_p            (d3_p),

    .lptxen_ln0      (1'b0),
    .lptxen_ln1      (1'b0),
    .lptxen_ln2      (1'b0),
    .lptxen_ln3      (1'b0),
    .lptxen_lnck     (1'b0),
    .do_lptx0_n      (1'b0),
    .do_lptx1_n      (1'b0),
    .do_lptx2_n      (1'b0),
    .do_lptx3_n      (1'b0),
    .do_lptxck_n     (1'b0),
    .do_lptx0_p      (1'b0),
    .do_lptx1_p      (1'b0),
    .do_lptx2_p      (1'b0),
    .do_lptx3_p      (1'b0),
    .do_lptxck_p     (1'b0),

    .hsrx_en_ck      (1'b1),
    .hsrx_en_d0      (1'b1),
    .hsrx_en_d1      (1'b1),
    .hsrx_en_d2      (1'b1),
    .hsrx_en_d3      (1'b1),
    .hsrx_odten_ck   (1'b1),
    .hsrx_odten_d0   (odt_en[0]),
    .hsrx_odten_d1   (odt_en[1]),
    .hsrx_odten_d2   (odt_en[2]),
    .hsrx_odten_d3   (odt_en[3]),
    .lprx_en_ck      (1'b1),
    .lprx_en_d0      (1'b1),
    .lprx_en_d1      (1'b1),
    .lprx_en_d2      (1'b1),
    .lprx_en_d3      (1'b1),
    .rx_drst_n       (rx_drst_n)
);

// Generate byte_ready similarly to the verified reference design.
reg        hsrx_en_msk = 1'b0;
reg [5:0]  hsrx_cnt    = 6'd0;
reg        byte_ready  = 1'b0;
reg [7:0]  byte_d0     = 8'd0;
reg [7:0]  byte_d1     = 8'd0;
reg [7:0]  byte_d2     = 8'd0;
reg [7:0]  byte_d3     = 8'd0;
wire [3:0] hsrxd_vld   = {d3ln_hsrxd_vld, d2ln_hsrxd_vld, d1ln_hsrxd_vld, d0ln_hsrxd_vld};

always @(posedge byte_clk or negedge rst_n_50m) begin
    if (!rst_n_50m) begin
        hsrx_cnt    <= 6'd0;
        r_lp0_0     <= 2'd0;
        r_lp0_1     <= 2'd0;
        rx_drst_n   <= 1'b1;
        hsrx_en_msk <= 1'b0;
        byte_ready  <= 1'b0;
        byte_d0     <= 8'd0;
        byte_d1     <= 8'd0;
        byte_d2     <= 8'd0;
        byte_d3     <= 8'd0;
    end else begin
        if (odt_en[1:0] != 2'b00)
            hsrx_cnt <= 6'd10;
        else if (hsrx_cnt > 6'd0)
            hsrx_cnt <= hsrx_cnt - 1'b1;

        r_lp0_0     <= lp_data0;
        r_lp0_1     <= r_lp0_0;
        rx_drst_n   <= ~(r_3to1 & from1to0);
        hsrx_en_msk <= (hsrx_cnt > 6'd0);

        byte_ready  <= hsrx_en_msk & (&hsrxd_vld);
        byte_d0     <= d0ln_hsrxd;
        byte_d1     <= d1ln_hsrxd;
        byte_d2     <= d2ln_hsrxd;
        byte_d3     <= d3ln_hsrxd;
    end
end

// --------------------------------------------------------------------------
// CSI-2 RX
// --------------------------------------------------------------------------
wire        csi_sp_en;
wire        csi_lp_en;
wire        csi_lp_av_en;
wire        csi_ecc_ok;
wire [7:0]  csi_ecc;
wire [15:0] csi_wc;
wire [1:0]  csi_vc;
wire [5:0]  csi_dt;
wire [31:0] csi_payload;
wire [3:0]  csi_payload_dv;

MIPI_DSI_CSI2_RX_Top u_mipi_csi2_rx (
    .I_RSTN        (rst_n_50m),
    .I_BYTE_CLK    (byte_clk),
    .I_REF_DT      (6'h1E),       // YUV422 8-bit
    .I_READY       (byte_ready),
    .I_DATA0       (byte_d0),
    .I_DATA1       (byte_d1),
    .I_DATA2       (byte_d2),
    .I_DATA3       (byte_d3),
    .O_SP_EN       (csi_sp_en),
    .O_LP_EN       (csi_lp_en),
    .O_LP_AV_EN    (csi_lp_av_en),
    .O_ECC_OK      (csi_ecc_ok),
    .O_ECC         (csi_ecc),
    .O_WC          (csi_wc),
    .O_VC          (csi_vc),
    .O_DT          (csi_dt),
    .O_PAYLOAD     (csi_payload),
    .O_PAYLOAD_DV  (csi_payload_dv)
);

// --------------------------------------------------------------------------
// Byte-to-pixel converter
// --------------------------------------------------------------------------
wire        cam_fv;
wire        cam_lv;
wire [31:0] cam_yuv422;

MIPI_Byte_to_Pixel_Converter_Top u_mipi_byte_to_pixel (
    .I_RSTN        (rst_n_50m),
    .I_BYTE_CLK    (byte_clk),
    .I_PIXEL_CLK   (byte_clk),
    .I_SP_EN       (csi_sp_en),
    .I_LP_AV_EN    (csi_lp_av_en),
    .I_DT          (csi_dt),
    .I_WC          (csi_wc),
    .I_PAYLOAD_DV  (csi_payload_dv),
    .I_PAYLOAD     (csi_payload),
    .O_FV          (cam_fv),
    .O_LV          (cam_lv),
    .O_PIXEL       (cam_yuv422)
);

// --------------------------------------------------------------------------
// YUV422 -> RGB888
// This converter outputs two RGB888 pixels per valid cycle.
// --------------------------------------------------------------------------
wire        rgb_fv;
wire        rgb_lv;
wire        rgb_de;
wire [47:0] rgb888_pair;

MIPI_YUV422toRGB888 u_yuv422_to_rgb888 (
    .mipi_clk      (byte_clk),
    .mipi_rstn     (rst_n_50m),
    .mipi_vsync    (cam_fv),
    .mipi_hsync    (cam_lv),
    .mipi_de       (cam_lv),
    .mipi_yuv422_i (cam_yuv422),
    .rgb_vsync     (rgb_fv),
    .rgb_hsync     (rgb_lv),
    .rgb_de        (rgb_de),
    .rgb888_o      (rgb888_pair)
);

// --------------------------------------------------------------------------
// Frame monitor
// --------------------------------------------------------------------------
wire [31:0] mon_frame_cnt;
wire [15:0] mon_line_cnt_cur;
wire [15:0] mon_line_cnt_last;
wire [15:0] mon_pixel_cnt_cur;
wire [15:0] mon_pixel_cnt_last;
wire [15:0] mon_pixel_cnt_max;
wire        mon_frame_seen;
wire        mon_line_seen;
wire        mon_pixel_seen;

camera_frame_monitor u_camera_frame_monitor (
    .clk             (byte_clk),
    .rst_n           (rst_n_50m),
    .fv              (rgb_fv),
    .lv              (rgb_lv),
    .de              (rgb_de),
    .pixel_inc       (2'd2),      // rgb_de carries two pixels in rgb888_pair
    .frame_cnt       (mon_frame_cnt),
    .line_cnt_cur    (mon_line_cnt_cur),
    .line_cnt_last   (mon_line_cnt_last),
    .pixel_cnt_cur   (mon_pixel_cnt_cur),
    .pixel_cnt_last  (mon_pixel_cnt_last),
    .pixel_cnt_max   (mon_pixel_cnt_max),
    .frame_seen      (mon_frame_seen),
    .line_seen       (mon_line_seen),
    .pixel_seen      (mon_pixel_seen)
);

// --------------------------------------------------------------------------
// LED: high-active.
//   slow blink : I2C not done
//   fast blink : I2C done, waiting for MIPI pixels
//   solid      : RGB pixels seen
// --------------------------------------------------------------------------
reg [25:0] led_cnt_50m = 26'd0;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        led_cnt_50m <= 26'd0;
    else
        led_cnt_50m <= led_cnt_50m + 1'b1;
end

assign led = !i2c_config_done ? led_cnt_50m[25] :
             !mon_pixel_seen  ? led_cnt_50m[22] :
                                1'b1;

// --------------------------------------------------------------------------
// ILA group #0: 50MHz / I2C domain
// Suggested GAO clock: clk
// Trigger 1: ila15i2c_done == 1
// Trigger 2: ila15i2c_index != 0
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15i2c_top_version = 32'h15B3_0001;
(* keep = "true" *) wire        ila15i2c_rst_n_50m   = rst_n_50m;
(* keep = "true" *) wire        ila15i2c_scl         = i2c_sclk;
(* keep = "true" *) wire        ila15i2c_sda         = i2c_sdat;
(* keep = "true" *) wire [8:0]  ila15i2c_index       = i2c_config_index;
(* keep = "true" *) wire [7:0]  ila15i2c_lut_size    = i2c_lut_size;
(* keep = "true" *) wire [23:0] ila15i2c_lut_data    = i2c_lut_data;
(* keep = "true" *) wire        ila15i2c_done        = i2c_config_done;

// --------------------------------------------------------------------------
// ILA group #1: MIPI byte clock / camera pixel domain
// Suggested GAO clock: byte_clk
// Trigger 1: ila15cam_rgb_de == 1
// Trigger 2: ila15cam_csi_lp_av_en == 1
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15cam_top_version       = 32'h15B3_0101;

// DPHY / byte lane status
(* keep = "true" *) wire        ila15cam_byte_clk_alive    = 1'b1;
(* keep = "true" *) wire [3:0]  ila15cam_hsrxd_vld         = hsrxd_vld;
(* keep = "true" *) wire        ila15cam_byte_ready        = byte_ready;
(* keep = "true" *) wire [7:0]  ila15cam_byte_d0           = byte_d0;
(* keep = "true" *) wire [7:0]  ila15cam_byte_d1           = byte_d1;
(* keep = "true" *) wire [7:0]  ila15cam_byte_d2           = byte_d2;
(* keep = "true" *) wire [7:0]  ila15cam_byte_d3           = byte_d3;
(* keep = "true" *) wire [3:0]  ila15cam_odt_en            = odt_en;
(* keep = "true" *) wire [1:0]  ila15cam_lp_data0          = lp_data0;
(* keep = "true" *) wire [1:0]  ila15cam_lp_clk            = lp_clk;

// CSI-2 status
(* keep = "true" *) wire        ila15cam_csi_sp_en          = csi_sp_en;
(* keep = "true" *) wire        ila15cam_csi_lp_en          = csi_lp_en;
(* keep = "true" *) wire        ila15cam_csi_lp_av_en       = csi_lp_av_en;
(* keep = "true" *) wire        ila15cam_csi_ecc_ok         = csi_ecc_ok;
(* keep = "true" *) wire [15:0] ila15cam_csi_wc             = csi_wc;
(* keep = "true" *) wire [5:0]  ila15cam_csi_dt             = csi_dt;
(* keep = "true" *) wire [31:0] ila15cam_csi_payload        = csi_payload;
(* keep = "true" *) wire [3:0]  ila15cam_csi_payload_dv     = csi_payload_dv;

// Byte-to-pixel output
(* keep = "true" *) wire        ila15cam_fv                 = cam_fv;
(* keep = "true" *) wire        ila15cam_lv                 = cam_lv;
(* keep = "true" *) wire [31:0] ila15cam_yuv422             = cam_yuv422;

// RGB output
(* keep = "true" *) wire        ila15cam_rgb_fv             = rgb_fv;
(* keep = "true" *) wire        ila15cam_rgb_lv             = rgb_lv;
(* keep = "true" *) wire        ila15cam_rgb_de             = rgb_de;
(* keep = "true" *) wire [23:0] ila15cam_rgb0               = rgb888_pair[23:0];
(* keep = "true" *) wire [23:0] ila15cam_rgb1               = rgb888_pair[47:24];

// Frame monitor
(* keep = "true" *) wire [31:0] ila15cam_frame_cnt          = mon_frame_cnt;
(* keep = "true" *) wire [15:0] ila15cam_line_cnt_cur       = mon_line_cnt_cur;
(* keep = "true" *) wire [15:0] ila15cam_line_cnt_last      = mon_line_cnt_last;
(* keep = "true" *) wire [15:0] ila15cam_pixel_cnt_cur      = mon_pixel_cnt_cur;
(* keep = "true" *) wire [15:0] ila15cam_pixel_cnt_last     = mon_pixel_cnt_last;
(* keep = "true" *) wire [15:0] ila15cam_pixel_cnt_max      = mon_pixel_cnt_max;
(* keep = "true" *) wire        ila15cam_frame_seen         = mon_frame_seen;
(* keep = "true" *) wire        ila15cam_line_seen          = mon_line_seen;
(* keep = "true" *) wire        ila15cam_pixel_seen         = mon_pixel_seen;

endmodule
