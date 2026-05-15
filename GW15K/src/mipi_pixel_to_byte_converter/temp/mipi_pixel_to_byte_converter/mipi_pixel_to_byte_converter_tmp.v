//Copyright (C)2014-2026 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12.02_SP2 (64-bit)
//IP Version: 1.1
//Part Number: GW5AT-LV15MG132C1/I0
//Device: GW5AT-15
//Device Version: B
//Created Time: Fri May 15 14:29:40 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	MIPI_Pixel_to_Byte_Converter_Top your_instance_name(
		.I_RSTN(I_RSTN), //input I_RSTN
		.I_PIXEL_CLK(I_PIXEL_CLK), //input I_PIXEL_CLK
		.I_BYTE_CLK(I_BYTE_CLK), //input I_BYTE_CLK
		.I_FV(I_FV), //input I_FV
		.I_LV(I_LV), //input I_LV
		.I_PIXEL(I_PIXEL), //input [31:0] I_PIXEL
		.O_FV_START(O_FV_START), //output O_FV_START
		.O_FV_END(O_FV_END), //output O_FV_END
		.O_DATA_EN(O_DATA_EN), //output O_DATA_EN
		.O_DATA(O_DATA) //output [31:0] O_DATA
	);

//--------Copy end-------------------
