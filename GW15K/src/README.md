# B3-1-fix2 双端逻辑：15K 摄像头 YUV422 -> RoraLink -> 60K YUV422 转 RGB -> DDR -> HDMI

## 目标

解决 B3-1/B3-1-fix1 中 15K 端 RGB888_32 扩展导致的吞吐问题：

- 摄像头原始输出是 YUV422，1 个 32bit word = 2 个像素。
- 之前 15K 端先转 RGB888_32，变成 1 个像素 1 个 32bit word，数据量扩大 2 倍。
- fix2 改成 15K 直接传 YUV422 pair，60K 接收后再转换成 RGB888_32 写 DDR。

这样 `Camera_Packet_FIFO_33` 不需要加宽，仍然使用：

```verilog
{last, data[31:0]}
```

## 15K 端替换/新增文件

放到 `GW15K/src/`：

```text
top.v
camera_crop_1080p_to_720p_yuv422_pair.v
camera_yuv422_packet_builder_720p.v
camera_packet_fifo_tx_reader.v
camera_frame_monitor.v
async_fifo_gray.v
```

继续保留工程里已有文件/IP：

```text
Camera_Packet_FIFO_33           // 33bit, depth 8192, FWFT, Wnum/Rnum enabled
gorwin_mipi_dphy / mipi_dsi_csi2_rx / mipi_byte_to_pixel_converter
I2C_ISPCAMERA_4Lanes_Config.v
i2c_timing_ctrl_reg16_dat8_wronly.v
roralink_tx_wrapper.v
serdes/...
```

`MIPI_YUV422toRGB888.v` 在 15K 主链路里不再使用，可以保留在工程里，但会被优化掉。

## 60K 端替换文件

放到 `GW60K/src/`：

```text
top.v
roralink_video_to_ddr_hdmi_b2.v
```

其余 DDR、HDMI、RoraLink RX wrapper、FIFO IP 继续沿用 B2-DB-fix2 工程。

## 新包格式

每个 segment packet：

```text
word0: 32'hA55A_6002
word1: {frame_id[15:0], line_id[10:0], segment_id[4:0]}
word2: {format[7:0], x_base[11:0], payload_words[11:0]}
word3: {8'h00, width[11:0], height[11:0]}
payload: 128 个 YUV422 pair word
```

关键参数：

```text
format        = 8'h02
x_base        = segment_id * 256 pixels
payload_words = 128
每个 payload word = 2 pixels YUV422
每行 5 个 segment
每帧 720 行
```

60K 端接收后：

```text
1 个 YUV422 word -> 2 个 RGB888_32 pixel word
4 个 YUV422 word -> 8 个 RGB888_32 word -> 1 个 256bit DDR write FIFO beat
```

## 上板后建议 ILA

### 15K byte_clk 域

采样时钟：`byte_clk`

触发条件优先：

```text
ila15cam_crop_sof == 1
```

异常触发：

```text
ila15cam_builder_pair_overflow_seen == 1
ila15cam_builder_drop_frame_seen == 1
```

重点看：

```text
ila15cam_crop_valid
ila15cam_crop_sof
ila15cam_crop_eof
ila15cam_crop_line_id
ila15cam_crop_pair_x
ila15cam_crop_yuv422
ila15cam_builder_state
ila15cam_builder_packet_wren
ila15cam_builder_packet_data
ila15cam_pair_fifo_wcnt
ila15cam_pair_fifo_rcnt
ila15cam_packet_fifo_wnum
ila15cam_builder_pair_overflow_seen
ila15cam_builder_packet_full_seen
ila15cam_builder_drop_frame_seen
```

### 15K tx_clk 域

采样时钟：`tx_clk`

触发条件：

```text
ila15tx_user_tx_valid == 1
```

重点看：

```text
ila15tx_user_tx_data
ila15tx_user_tx_last
ila15tx_packet_cnt
ila15tx_fifo_rnum
ila15tx_fifo_underflow_seen
```

正常预期：

```text
ila15tx_fifo_underflow_seen = 0
ila15cam_builder_pair_overflow_seen = 0
ila15cam_builder_packet_full_seen = 0
```

### 60K RX 域

采样时钟：`rl_rx_clk`

触发条件：

```text
ila60b2_rl_rx_valid == 1
```

重点看：

```text
ila60b2_rl_rx_data
ila60b2_rx_packet_cnt
ila60b2_rx_crc_pass_cnt
ila60b2_rx_word_idx
ila60b2_rx_line_id
ila60b2_rx_seg_id
ila60b2_rx_header_err_seen
ila60b2_rx_last_err_seen
ila60b2_rx_overrun_err_seen
```

## 备注

这版 60K 接收桥默认接收 `format=8'h02` 的 YUV422 packet；如果你再用旧的 15K RGB colorbar 发送端，60K 会判 header format 不匹配。需要回到 colorbar 测试时，请换回 B2-DB-fix2 的 60K bridge，或者给 60K bridge 增加 RGB/YUV 双格式兼容。
