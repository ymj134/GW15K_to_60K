//Copyright (C)2014-2026 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12.02_SP2 (64-bit)
//IP Version: 1.1
//Part Number: GW5AT-LV60PG484AC2/I1
//Device: GW5AT-60
//Device Version: B
//Created Time: Thu Apr 30 15:24:46 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	Video_FIFO_256to32 your_instance_name(
		.Data(Data), //input [255:0] Data
		.Reset(Reset), //input Reset
		.WrClk(WrClk), //input WrClk
		.RdClk(RdClk), //input RdClk
		.WrEn(WrEn), //input WrEn
		.RdEn(RdEn), //input RdEn
		.Wnum(Wnum), //output [10:0] Wnum
		.Rnum(Rnum), //output [13:0] Rnum
		.Almost_Empty(Almost_Empty), //output Almost_Empty
		.Almost_Full(Almost_Full), //output Almost_Full
		.Q(Q), //output [31:0] Q
		.Empty(Empty), //output Empty
		.Full(Full) //output Full
	);

//--------Copy end-------------------
