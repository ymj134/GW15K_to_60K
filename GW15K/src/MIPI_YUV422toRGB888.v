
module	MIPI_YUV422toRGB888(
	
	input					mipi_clk,
	input					mipi_rstn,
	input					mipi_vsync,
	input					mipi_hsync,
	input					mipi_de,
	input		[31:0]		mipi_yuv422_i,
	
	output					rgb_vsync,
	output					rgb_hsync,
	output					rgb_de,
	output		[47:0]		rgb888_o



);


	reg					p1_flag;
	reg    	[19:0]		Y1,U1,V1,Y2,U2,V2;
	always@(posedge	mipi_clk or negedge mipi_rstn)begin
	if(!mipi_rstn)begin
		Y1 <= 0; Y2 <= 0; U1 <= 0; U2 <= 0; V1 <= 0; V2 <= 0;
		p1_flag <= 0;
	end
	else begin
		if(mipi_vsync)begin
			if(mipi_hsync & mipi_de)begin
				Y1 <= mipi_yuv422_i[15:8]*596;
				Y2 <= mipi_yuv422_i[31:24]*596;
				U1 <= mipi_yuv422_i[7:0]*200;
				U2 <= mipi_yuv422_i[7:0]*1033;
				V1 <= mipi_yuv422_i[23:16]*817;
				V2 <= mipi_yuv422_i[23:16]*416;
				p1_flag <= 1;
			end
			else begin
				Y1 <= Y1; Y2 <= Y2; U1 <= U1; U2 <= U2; V1 <= V1; V2 <= V2;
				p1_flag <=0;
			end
		end
		else begin
			Y1 <= 0; Y2 <= 0; U1 <= 0; U2 <= 0; V1 <= 0; V2 <= 0;
			p1_flag <= 0;
		end
	end
	end
	
	reg		[19:0]		pix0r_p1,pix0g_p1,pix0b_p1;
	reg		[19:0]		pix1r_p1,pix1g_p1,pix1b_p1;
	reg					r_data_vld;
	always@(posedge	mipi_clk or negedge mipi_rstn)begin
	if(!mipi_rstn)begin
		pix0r_p1 <= 0;
		pix0g_p1 <= 0;
		pix0b_p1 <= 0;
		pix1r_p1 <= 0;
		pix1g_p1 <= 0;
		pix1b_p1 <= 0;
		r_data_vld <= 0;
	end
	else if(p1_flag)begin
		pix0r_p1 <= (Y1 + V1 - 114131) >> 9;
		pix0g_p1 <= (Y1 - U1 - V2 + 69370) >> 9;
		pix0b_p1 <= (Y1 + U2 - 141787) >> 9;
		pix1r_p1 <= (Y2 + V1 - 114131) >> 9;
		pix1g_p1 <= (Y2 - U1 - V2 + 69370) >> 9;
		pix1b_p1 <= (Y2 + U2 - 141787) >> 9;
		r_data_vld <= 1;
	end
	else begin
		pix0r_p1 <= 0;
		pix0g_p1 <= 0;
		pix0b_p1 <= 0;
		pix1r_p1 <= 0;
		pix1g_p1 <= 0;
		pix1b_p1 <= 0;
		r_data_vld <= 0;
	end
	end
	
	reg		[1:0]		r_vs, r_hs;
	always@(posedge mipi_clk)begin
		r_vs <= {r_vs,mipi_vsync};
		r_hs <= {r_hs,mipi_hsync};
	end
	
	wire	[7:0]	p1_r,p1_g,p1_b;
	wire	[7:0]	p2_r,p2_g,p2_b;
	
	assign rgb_vsync = r_vs[1];
	assign rgb_hsync = r_hs[1];
	assign rgb_de = r_data_vld;
	assign rgb888_o = {p2_r,p2_g,p2_b,p1_r,p1_g,p1_b};

	assign p1_r = pix0r_p1[10]?8'd0:((pix0r_p1[9:0]>'d255)?8'd255:pix0r_p1[7:0]);
	assign p1_g = pix0g_p1[10]?8'd0:((pix0g_p1[9:0]>'d255)?8'd255:pix0g_p1[7:0]);
	assign p1_b = pix0b_p1[10]?8'd0:((pix0b_p1[9:0]>'d255)?8'd255:pix0b_p1[7:0]);
	assign p2_r = pix1r_p1[10]?8'd0:((pix1r_p1[9:0]>'d255)?8'd255:pix1r_p1[7:0]);
	assign p2_g = pix1g_p1[10]?8'd0:((pix1g_p1[9:0]>'d255)?8'd255:pix1g_p1[7:0]);
	assign p2_b = pix1b_p1[10]?8'd0:((pix1b_p1[9:0]>'d255)?8'd255:pix1b_p1[7:0]);
	
endmodule