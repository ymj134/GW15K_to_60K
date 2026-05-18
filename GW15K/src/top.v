// ============================================================================
// GW15K B3-1-fix2 Camera 1080p YUV422 -> center-crop 720p -> RoraLink TX top
// ----------------------------------------------------------------------------
// Data path:
//   Camera I2C config
//      -> Gowin_MIPI_DPHY RX 4-lane
//      -> MIPI_DSI_CSI2_RX_Top
//      -> MIPI_Byte_to_Pixel_Converter_Top
//      -> 1920x1080 center-crop to 1280x720 in native YUV422 pairs
//      -> camera_yuv422_packet_builder_720p
//      -> Camera_Packet_FIFO_33, byte_clk to RoraLink tx_clk
//      -> camera_packet_fifo_tx_reader
//      -> RoraLink TX-only IP
//
// Notes:
//   1) Use the matched 60K B3-1-fix2 receiver bridge in this package.
//   2) Add camera/MIPI/I2C pin constraints in fpga_project.cst.
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
// Frame monitor on raw Byte-to-Pixel YUV422 stream
// O_PIXEL is one YUV422 pair per byte_clk when FV/LV are active.
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
    .fv              (cam_fv),
    .lv              (cam_lv),
    .de              (cam_lv),
    .pixel_inc       (2'd2),      // one YUV422 32-bit word carries two pixels
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
// 1080p camera YUV422 pair -> centered 720p YUV422 pair crop
// --------------------------------------------------------------------------
wire        crop_valid;
wire [31:0] crop_yuv422;
wire        crop_sof;
wire        crop_eol;
wire        crop_eof;
wire [10:0] crop_line_id;
wire [10:0] crop_pair_x;
wire        crop_frame_seen;
wire [31:0] crop_frame_cnt;
wire [15:0] crop_pair_cnt;
wire [15:0] crop_line_cnt;

camera_crop_1080p_to_720p_yuv422_pair #(
    .IN_H_RES     (12'd1920),
    .IN_V_RES     (11'd1080),
    .CROP_X_START (12'd320),
    .CROP_Y_START (11'd180),
    .OUT_H_RES    (12'd1280),
    .OUT_V_RES    (11'd720)
) u_camera_crop_720p_yuv422 (
    .clk            (byte_clk),
    .rst_n          (rst_n_50m),
    .in_fv          (cam_fv),
    .in_lv          (cam_lv),
    .in_de          (cam_lv),
    .in_yuv422_pair (cam_yuv422),
    .out_valid      (crop_valid),
    .out_yuv422_pair(crop_yuv422),
    .out_sof        (crop_sof),
    .out_eol        (crop_eol),
    .out_eof        (crop_eof),
    .out_line_id    (crop_line_id),
    .out_pair_x     (crop_pair_x),
    .frame_seen     (crop_frame_seen),
    .crop_frame_cnt (crop_frame_cnt),
    .crop_pair_cnt  (crop_pair_cnt),
    .crop_line_cnt  (crop_line_cnt)
);

// --------------------------------------------------------------------------
// RoraLink TX wrapper
// --------------------------------------------------------------------------
wire        rl_sys_reset;
wire        user_tx_ready;
wire        hard_err;
wire        channel_up;
wire        lane_up;
wire        tx_clk;
wire        gt_pll_ok;
wire        tx_rst;
wire [31:0] user_tx_data;
wire [3:0]  user_tx_strb;
wire        user_tx_valid;
wire        user_tx_last;

roralink_tx_wrapper u_rl_tx_wrapper (
    .init_clk      (clk),
    .rst_n         (rst_n),
    .tx_clk        (tx_clk),
    .tx_rst        (tx_rst),
    .rl_sys_reset  (rl_sys_reset),
    .gt_pll_ok     (gt_pll_ok),
    .lane_up       (lane_up),
    .channel_up    (channel_up),
    .hard_err      (hard_err),
    .user_tx_ready (user_tx_ready),
    .user_tx_data  (user_tx_data),
    .user_tx_strb  (user_tx_strb),
    .user_tx_valid (user_tx_valid),
    .user_tx_last  (user_tx_last)
);



// --------------------------------------------------------------------------
// B3-1-fix2 packet-aware bridge, native YUV422 transport
//   byte_clk domain builds complete RoraLink packets with 128 YUV422-pair
//   payload words per 256-pixel segment.
//   Camera_Packet_FIFO_33 crosses byte_clk -> tx_clk.
// --------------------------------------------------------------------------
wire packet_fifo_rst = (~rst_n_50m) | tx_rst | (~channel_up);

// Synchronize RoraLink channel_up into the camera byte clock domain.
reg [2:0] channel_up_bsync = 3'b000;
always @(posedge byte_clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        channel_up_bsync <= 3'b000;
    else
        channel_up_bsync <= {channel_up_bsync[1:0], channel_up};
end
wire channel_up_byte = channel_up_bsync[2];

wire [32:0] packet_fifo_din;
wire        packet_fifo_wren;
wire        packet_fifo_rden;
wire [32:0] packet_fifo_q;
wire [13:0] packet_fifo_wnum;
wire [13:0] packet_fifo_rnum;
wire        packet_fifo_aempty;
wire        packet_fifo_afull;
wire        packet_fifo_empty;
wire        packet_fifo_full;

// Camera-domain YUV packet builder debug
wire        yuv_pair_fifo_full_dbg;
wire        yuv_pair_fifo_empty_dbg;
wire [15:0] yuv_pair_fifo_wr_count_dbg;
wire [15:0] yuv_pair_fifo_rd_count_dbg;
wire        yuv_pair_fifo_rd_en_dbg;
wire [3:0]  builder_state;
wire [15:0] builder_frame_id;
wire [10:0] builder_line_id;
wire [4:0]  builder_segment_id;
wire [7:0]  builder_pair_idx;
wire [11:0] builder_word_idx;
wire [31:0] builder_packet_cnt;
wire [31:0] builder_packet_word_cnt;
wire        builder_frame_active;
wire        builder_sof_seen;
wire        builder_packet_wr_seen;
wire        builder_packet_last_seen;
wire        builder_pair_overflow_seen;
wire        builder_pair_empty_seen;
wire        builder_packet_full_seen;
wire        builder_packet_afull_seen;
wire        builder_drop_frame_seen;

camera_yuv422_packet_builder_720p #(
    .H_RES                (12'd1280),
    .V_RES                (12'd720),
    .LAST_LINE            (11'd719),
    .LAST_SEG             (5'd4),
    .SEG_PIXELS           (12'd256),
    .PAIR_FIFO_ADDR_WIDTH (9),       // 512 YUV422-pair elasticity; enough for header overhead
    .START_PREFILL_PAIRS  (16'd64)
) u_camera_yuv422_packet_builder (
    .clk                         (byte_clk),
    .rst                         (packet_fifo_rst),
    .link_active                 (channel_up_byte),
    .crop_valid                  (crop_valid),
    .crop_sof                    (crop_sof),
    .crop_eof                    (crop_eof),
    .crop_yuv422                 (crop_yuv422),
    .packet_fifo_full            (packet_fifo_full),
    .packet_fifo_almost_full     (packet_fifo_afull),
    .packet_fifo_wr_en           (packet_fifo_wren),
    .packet_fifo_wr_data         (packet_fifo_din),
    .pair_fifo_full_dbg          (yuv_pair_fifo_full_dbg),
    .pair_fifo_empty_dbg         (yuv_pair_fifo_empty_dbg),
    .pair_fifo_wr_count_dbg      (yuv_pair_fifo_wr_count_dbg),
    .pair_fifo_rd_count_dbg      (yuv_pair_fifo_rd_count_dbg),
    .pair_fifo_rd_en_dbg         (yuv_pair_fifo_rd_en_dbg),
    .builder_state_dbg           (builder_state),
    .frame_id_dbg                (builder_frame_id),
    .line_id_dbg                 (builder_line_id),
    .segment_id_dbg              (builder_segment_id),
    .pair_idx_dbg                (builder_pair_idx),
    .word_idx_dbg                (builder_word_idx),
    .packet_cnt_dbg              (builder_packet_cnt),
    .packet_word_cnt_dbg         (builder_packet_word_cnt),
    .frame_active_dbg            (builder_frame_active),
    .sof_seen_dbg                (builder_sof_seen),
    .packet_wr_seen_dbg          (builder_packet_wr_seen),
    .packet_last_seen_dbg        (builder_packet_last_seen),
    .pair_fifo_overflow_seen_dbg (builder_pair_overflow_seen),
    .pair_fifo_empty_seen_dbg    (builder_pair_empty_seen),
    .packet_fifo_full_seen_dbg   (builder_packet_full_seen),
    .packet_fifo_afull_seen_dbg  (builder_packet_afull_seen),
    .drop_frame_seen_dbg         (builder_drop_frame_seen)
);

Camera_Packet_FIFO_33 u_camera_packet_fifo (
    .Data         (packet_fifo_din),
    .Reset        (packet_fifo_rst),
    .WrClk        (byte_clk),
    .RdClk        (tx_clk),
    .WrEn         (packet_fifo_wren),
    .RdEn         (packet_fifo_rden),
    .Wnum         (packet_fifo_wnum),
    .Rnum         (packet_fifo_rnum),
    .Almost_Empty (packet_fifo_aempty),
    .Almost_Full  (packet_fifo_afull),
    .Q            (packet_fifo_q),
    .Empty        (packet_fifo_empty),
    .Full         (packet_fifo_full)
);

wire        tx_fifo_fire;
wire [1:0]  tx_reader_state;
wire [31:0] tx_packet_cnt;
wire [31:0] tx_word_cnt;
wire        tx_start_seen;
wire        tx_fire_seen;
wire        tx_last_seen;
wire        tx_ready_seen;
wire        tx_channel_seen;
wire        tx_lane_seen;
wire        tx_hard_err_seen;
wire        tx_fifo_empty_seen;
wire        tx_fifo_underflow_seen;

camera_packet_fifo_tx_reader #(
    .PACKET_WORDS (14'd132)       // 4 header + 128 YUV422-pair payload words
) u_camera_packet_fifo_tx_reader (
    .clk                       (tx_clk),
    .rst                       (tx_rst),
    .channel_up                (channel_up),
    .lane_up                   (lane_up),
    .hard_err                  (hard_err),
    .tx_ready                  (user_tx_ready),
    .fifo_q                    (packet_fifo_q),
    .fifo_empty                (packet_fifo_empty),
    .fifo_rnum                 (packet_fifo_rnum),
    .fifo_rd_en                (packet_fifo_rden),
    .tx_data                   (user_tx_data),
    .tx_strb                   (user_tx_strb),
    .tx_valid                  (user_tx_valid),
    .tx_last                   (user_tx_last),
    .tx_fire_dbg               (tx_fifo_fire),
    .tx_state_dbg              (tx_reader_state),
    .packet_cnt_dbg            (tx_packet_cnt),
    .word_cnt_dbg              (tx_word_cnt),
    .start_seen_dbg            (tx_start_seen),
    .fire_seen_dbg             (tx_fire_seen),
    .last_seen_dbg             (tx_last_seen),
    .ready_seen_dbg            (tx_ready_seen),
    .channel_seen_dbg          (tx_channel_seen),
    .lane_seen_dbg             (tx_lane_seen),
    .hard_err_seen_dbg         (tx_hard_err_seen),
    .fifo_empty_seen_dbg       (tx_fifo_empty_seen),
    .fifo_underflow_seen_dbg   (tx_fifo_underflow_seen)
);

// --------------------------------------------------------------------------
// LED: high-active for 15K board LED.
// --------------------------------------------------------------------------
reg [25:0] led_cnt_50m = 26'd0;
always @(posedge clk or negedge rst_n_50m) begin
    if (!rst_n_50m)
        led_cnt_50m <= 26'd0;
    else
        led_cnt_50m <= led_cnt_50m + 1'b1;
end

assign led = (!mon_pixel_seen) ? led_cnt_50m[25] :
             (!tx_last_seen)   ? led_cnt_50m[22] :
             (builder_pair_overflow_seen || builder_packet_full_seen || builder_drop_frame_seen || tx_fifo_underflow_seen || tx_hard_err_seen) ? led_cnt_50m[21] :
                                  1'b1;

// --------------------------------------------------------------------------
// ILA group #0: 50MHz / I2C domain
// Suggested GAO clock: clk
// Trigger 1: ila15i2c_done == 1
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15i2c_top_version = 32'h15B3_1201;
(* keep = "true" *) wire        ila15i2c_rst_n_50m   = rst_n_50m;
(* keep = "true" *) wire        ila15i2c_scl         = i2c_sclk;
(* keep = "true" *) wire        ila15i2c_sda         = i2c_sdat;
(* keep = "true" *) wire [8:0]  ila15i2c_index       = i2c_config_index;
(* keep = "true" *) wire [7:0]  ila15i2c_lut_size    = i2c_lut_size;
(* keep = "true" *) wire [23:0] ila15i2c_lut_data    = i2c_lut_data;
(* keep = "true" *) wire        ila15i2c_done        = i2c_config_done;

// --------------------------------------------------------------------------
// ILA group #1: MIPI byte clock / camera crop + YUV packet-builder domain
// Suggested GAO clock: byte_clk
// Trigger 1: ila15cam_crop_sof == 1
// Trigger 2: ila15cam_builder_pair_overflow_seen == 1
// Trigger 3: ila15cam_builder_drop_frame_seen == 1
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15cam_top_version       = 32'h15B3_1201;
(* keep = "true" *) wire [3:0]  ila15cam_hsrxd_vld         = hsrxd_vld;
(* keep = "true" *) wire        ila15cam_byte_ready        = byte_ready;
(* keep = "true" *) wire [15:0] ila15cam_csi_wc            = csi_wc;
(* keep = "true" *) wire [5:0]  ila15cam_csi_dt            = csi_dt;
(* keep = "true" *) wire [31:0] ila15cam_csi_payload       = csi_payload;
(* keep = "true" *) wire [3:0]  ila15cam_csi_payload_dv    = csi_payload_dv;
(* keep = "true" *) wire        ila15cam_fv                = cam_fv;
(* keep = "true" *) wire        ila15cam_lv                = cam_lv;
(* keep = "true" *) wire [31:0] ila15cam_yuv422            = cam_yuv422;
(* keep = "true" *) wire [31:0] ila15cam_frame_cnt         = mon_frame_cnt;
(* keep = "true" *) wire [15:0] ila15cam_line_cnt_last     = mon_line_cnt_last;
(* keep = "true" *) wire [15:0] ila15cam_pixel_cnt_last    = mon_pixel_cnt_last;
(* keep = "true" *) wire        ila15cam_crop_valid        = crop_valid;
(* keep = "true" *) wire        ila15cam_crop_sof          = crop_sof;
(* keep = "true" *) wire        ila15cam_crop_eol          = crop_eol;
(* keep = "true" *) wire        ila15cam_crop_eof          = crop_eof;
(* keep = "true" *) wire [10:0] ila15cam_crop_line_id      = crop_line_id;
(* keep = "true" *) wire [10:0] ila15cam_crop_pair_x       = crop_pair_x;
(* keep = "true" *) wire [31:0] ila15cam_crop_yuv422       = crop_yuv422;
(* keep = "true" *) wire [15:0] ila15cam_crop_pair_cnt     = crop_pair_cnt;
(* keep = "true" *) wire [15:0] ila15cam_crop_line_cnt     = crop_line_cnt;
(* keep = "true" *) wire        ila15cam_channel_up_byte   = channel_up_byte;
(* keep = "true" *) wire [3:0]  ila15cam_builder_state     = builder_state;
(* keep = "true" *) wire [15:0] ila15cam_builder_frame_id  = builder_frame_id;
(* keep = "true" *) wire [10:0] ila15cam_builder_line_id   = builder_line_id;
(* keep = "true" *) wire [4:0]  ila15cam_builder_seg_id    = builder_segment_id;
(* keep = "true" *) wire [7:0]  ila15cam_builder_pair_idx  = builder_pair_idx;
(* keep = "true" *) wire [11:0] ila15cam_builder_word_idx  = builder_word_idx;
(* keep = "true" *) wire        ila15cam_builder_frame_active = builder_frame_active;
(* keep = "true" *) wire        ila15cam_builder_sof_seen  = builder_sof_seen;
(* keep = "true" *) wire        ila15cam_builder_packet_wren = packet_fifo_wren;
(* keep = "true" *) wire [32:0] ila15cam_builder_packet_data = packet_fifo_din;
(* keep = "true" *) wire [31:0] ila15cam_builder_packet_cnt = builder_packet_cnt;
(* keep = "true" *) wire [31:0] ila15cam_builder_word_cnt   = builder_packet_word_cnt;
(* keep = "true" *) wire        ila15cam_pair_fifo_full    = yuv_pair_fifo_full_dbg;
(* keep = "true" *) wire        ila15cam_pair_fifo_empty   = yuv_pair_fifo_empty_dbg;
(* keep = "true" *) wire        ila15cam_pair_fifo_rden    = yuv_pair_fifo_rd_en_dbg;
(* keep = "true" *) wire [15:0] ila15cam_pair_fifo_wcnt    = yuv_pair_fifo_wr_count_dbg;
(* keep = "true" *) wire [15:0] ila15cam_pair_fifo_rcnt    = yuv_pair_fifo_rd_count_dbg;
(* keep = "true" *) wire [13:0] ila15cam_packet_fifo_wnum  = packet_fifo_wnum;
(* keep = "true" *) wire        ila15cam_packet_fifo_full  = packet_fifo_full;
(* keep = "true" *) wire        ila15cam_packet_fifo_afull = packet_fifo_afull;
(* keep = "true" *) wire        ila15cam_builder_pair_overflow_seen = builder_pair_overflow_seen;
(* keep = "true" *) wire        ila15cam_builder_pair_empty_seen    = builder_pair_empty_seen;
(* keep = "true" *) wire        ila15cam_builder_packet_full_seen   = builder_packet_full_seen;
(* keep = "true" *) wire        ila15cam_builder_packet_afull_seen  = builder_packet_afull_seen;
(* keep = "true" *) wire        ila15cam_builder_drop_frame_seen    = builder_drop_frame_seen;

// --------------------------------------------------------------------------
// ILA group #2: RoraLink tx_clk / packet FIFO-read domain
// Suggested GAO clock: tx_clk
// Trigger 1: ila15tx_user_tx_valid == 1
// Trigger 2: ila15tx_fifo_underflow_seen == 1
// --------------------------------------------------------------------------
(* keep = "true" *) wire [31:0] ila15tx_top_version          = 32'h15B3_1201;
(* keep = "true" *) wire        ila15tx_gt_pll_ok            = gt_pll_ok;
(* keep = "true" *) wire        ila15tx_rl_sys_reset         = rl_sys_reset;
(* keep = "true" *) wire        ila15tx_tx_rst               = tx_rst;
(* keep = "true" *) wire        ila15tx_lane_up              = lane_up;
(* keep = "true" *) wire        ila15tx_channel_up           = channel_up;
(* keep = "true" *) wire        ila15tx_user_tx_ready        = user_tx_ready;
(* keep = "true" *) wire        ila15tx_user_tx_valid        = user_tx_valid;
(* keep = "true" *) wire        ila15tx_user_tx_last         = user_tx_last;
(* keep = "true" *) wire [31:0] ila15tx_user_tx_data         = user_tx_data;
(* keep = "true" *) wire [3:0]  ila15tx_user_tx_strb         = user_tx_strb;
(* keep = "true" *) wire        ila15tx_fire                 = tx_fifo_fire;
(* keep = "true" *) wire [1:0]  ila15tx_state                = tx_reader_state;
(* keep = "true" *) wire [31:0] ila15tx_packet_cnt           = tx_packet_cnt;
(* keep = "true" *) wire [31:0] ila15tx_word_cnt             = tx_word_cnt;
(* keep = "true" *) wire        ila15tx_start_seen           = tx_start_seen;
(* keep = "true" *) wire        ila15tx_last_seen            = tx_last_seen;
(* keep = "true" *) wire        ila15tx_hard_err             = hard_err;
(* keep = "true" *) wire        ila15tx_hard_err_seen        = tx_hard_err_seen;
(* keep = "true" *) wire        ila15tx_fifo_empty           = packet_fifo_empty;
(* keep = "true" *) wire [13:0] ila15tx_fifo_rnum            = packet_fifo_rnum;
(* keep = "true" *) wire        ila15tx_fifo_rden            = packet_fifo_rden;
(* keep = "true" *) wire [32:0] ila15tx_fifo_q               = packet_fifo_q;
(* keep = "true" *) wire        ila15tx_fifo_empty_seen      = tx_fifo_empty_seen;
(* keep = "true" *) wire        ila15tx_fifo_underflow_seen  = tx_fifo_underflow_seen;

endmodule
