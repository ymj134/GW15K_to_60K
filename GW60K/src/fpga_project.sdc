//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12.02_SP2 (64-bit) 
//Created Time: 2026-05-11 14:29:13
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

//6.25G的约束
//create_clock -name rl_rx_clk -period 6.4 -waveform {0 3.2} [get_nets {rl_rx_clk}]


//3.125G的约束
create_clock -name rl_rx_clk -period 12.8-waveform {0 6.4} [get_nets {rl_rx_clk}]