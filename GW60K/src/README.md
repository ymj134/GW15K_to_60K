# GW60K modular 720p B2 split

This package is a first-pass structural split of the current monolithic `GW60K/src/top.v`.

Functional intent is unchanged:

- 15K RoraLink segmented RGB888_32 RX
- RX 32-bit pixel payload pack to 256-bit beat
- RX clock to DDR `clk_out` through `Video_WR_FIFO_256`
- AXI write to DDR framebuffer
- AXI read from DDR framebuffer
- DDR `clk_out` to HDMI `pixel_clk` through `Video_FIFO_256to32`
- ADV7513 720p60 output

Files:

- `top.v` — board-level top, connections, LED, ILA signals only
- `clock_reset_60k.v` — 50MHz reset sync, pixel PLL, DDR PLL/MDPR glue
- `ddr3_axi_wrapper_60k.v` — DDR3_Memory_Interface_Top wrapper
- `roralink_rx_wrapper.v` — SerDes_Top / RoraLink RX-only wrapper
- `hdmi_720p_timing.v` — 720p60 timing generator
- `hdmi_output_stage.v` — registered HDMI output RGB/HS/VS/DE stage
- `adv7513_iic_wrapper.v` — ADV7513 I2C initialization wrapper
- `roralink_video_to_ddr_hdmi_b2.v` — original B2 bridge moved out unchanged internally

Add all `.v` files to the GW60K project and use this `top.v` as the top module.
