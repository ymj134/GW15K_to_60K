//Copyright (C)2014-2026 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12 (64-bit) 
//Created Time: 2026-04-23 16:27:28
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
//create_clock -name gt_pcs_tx_clk -period 12.8 -waveform {0 6.4} [get_nets {gt_pcs_tx_clk}]
//create_clock -name gt_pcs_rx_clk -period 12.8 -waveform {0 6.4} [get_nets {gt_pcs_rx_clk}]
