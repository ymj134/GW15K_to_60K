# GW60K B3-1-fix2-rx-capture-fix

Replace `GW60K/src/roralink_video_to_ddr_hdmi_b2.v` with the provided file.

## What changed

The RX parser now decides whether to accept a packet at header word3 instead of word1.

Old behavior:
- word0 checked MAGIC
- word1 latched line/segment and immediately tried to start capture

New behavior:
- word0 checks MAGIC
- word1 latches frame/line/segment
- word2 latches format/x_base/payload_words
- word3 checks width/height and opens `capture_this_pkt` for word4 payload

This should fix the condition where 60K receives a valid `A55A6002 / 02000080 / 005002D0` packet but `rx_capture_active`, `rx_payload_accept`, and `rx_wr_fifo_wren` remain 0.

## Suggested ILA after programming

RX clock domain: `rl_rx_clk`
Trigger: `ila60b2_rl_rx_valid == 1 && ila60b2_rl_rx_data == 32'hA55A6002`

Expected after fix:
- `ila60b2_rx_frame_accept_allowed = 1` at line0/seg0
- `ila60b2_rx_capture_active` becomes 1 after word3
- `ila60b2_rx_payload_accept` pulses during payload
- `ila60b2_rx_wr_fifo_wren` pulses once every 4 YUV422 payload words
- `ila60b2_wr_fifo_wr_count` increases
- `ila60b2_first_frame_written` eventually becomes 1
