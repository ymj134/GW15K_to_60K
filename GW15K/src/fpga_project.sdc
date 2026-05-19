//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12.02_SP2 (64-bit) 
//Created Time: 2026-05-19 11:01:34
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
create_clock -name rl_tx_clk -period 6.4 -waveform {0 3.2} [get_nets {tx_clk}]
create_clock -name byte_clk -period 13.468 -waveform {0 6.734} [get_nets {byte_clk}]
set_clock_groups -asynchronous -group [get_clocks {clk}] -group [get_clocks {rl_tx_clk}] -group [get_clocks {byte_clk}]
