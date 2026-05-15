GW60K 720p B2-DB-fix2 modified files
=====================================

Replace only these files in GW60K/src:

- top.v
- roralink_video_to_ddr_hdmi_b2.v

Main change from B2-DB-fix1
---------------------------

The RX side is now frame-aware:

1. At the start of each incoming frame (line_id=0, seg_id=0), the RX parser checks whether the AXI/DDR writer has a free framebuffer and whether the write FIFO is near empty.
2. If a free buffer is available, the whole frame is accepted and written to DDR.
3. If no free buffer is available, the whole incoming frame is dropped at the packet layer.
4. This prevents Video_WR_FIFO_256 from overflowing and prevents a partial frame from shifting framebuffer address 0 into the middle of the colorbar.

Version marker
--------------

ila60b2_top_version = 32'h60B2_DB12

New debug signals
-----------------

- ila60b2_rx_frame_accept_allowed
- ila60b2_rx_drop_frame_active

Recommended quick check
-----------------------

Use RX-domain ILA with rl_rx_clk. Trigger on:

    ila60b2_rx_overrun_err_seen == 1

Expected after fix2:

- ila60b2_rx_overrun_err_seen should stay 0.
- ila60b2_rx_drop_frame_active may pulse when no free DDR buffer is available.
- HDMI x=0 should return to FFFFFF for the leftmost white colorbar.
