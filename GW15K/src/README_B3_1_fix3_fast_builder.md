# GW15K B3-1-fix3 fast YUV422 packet builder

Replace:

```text
GW15K/src/camera_yuv422_packet_builder_720p.v
```

with the file in this package.

This version keeps the same module interface. `top.v` does not need to change.

Main fix:

- The previous builder consumed one YUV422 pair in three byte_clk cycles because it used `REQ_PAIR -> LOAD_PAIR -> PAYLOAD` around a registered pair FIFO.
- The camera crop stream can output one YUV422 pair every byte_clk.
- This mismatch caused `builder_pair_overflow_seen=1` and corrupted/blocked video.
- The new builder uses a same-clock FWFT-style elastic FIFO and writes one payload word per byte_clk during payload.
- It no longer gates frame start with `packet_fifo_almost_full`; it records AFULL for debug only.

After programming, in the 15K byte_clk ILA expect:

```text
builder_pair_overflow_seen = 0
packet_fifo_full_seen      = 0
pair_fifo_full             = 0
packet_fifo_wren           pulses
pair_fifo_rden             pulses during payload
```

`packet_fifo_afull_seen` may still become 1 depending on the FIFO AFULL threshold; it should not block capture anymore.
