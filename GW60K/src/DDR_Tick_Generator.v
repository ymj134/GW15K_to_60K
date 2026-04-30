//by CrazyStaff		20241009      v1.0

module DDR_Tick_Generator#(
    parameter       CLK_PERIOD = 48_000_000
)(
	input				sys_clk_i,
	input				sys_rstn_i,
	
	output   			ustick_o,  // 1MHz
	output   			mstick_o	// 1KHz

);

	localparam	US_DELAY = CLK_PERIOD/1_000_000,
				MS_DELAY = CLK_PERIOD/1_000;
				
	reg		[11:0]		ustick_cnt;
	reg		[15:0]		mstick_cnt;
	
	reg					r_ustick;
	reg					r_mstick;
	
	always@(posedge sys_clk_i or negedge sys_rstn_i)begin
	if(~sys_rstn_i)begin
		ustick_cnt <= 0;
	end
	else if(ustick_cnt == US_DELAY - 1'b1)begin
		ustick_cnt <= 0;
	end
	else begin
		ustick_cnt <= ustick_cnt + 1'b1;
	end
	end
	
	always@(posedge sys_clk_i or negedge sys_rstn_i)begin
	if(~sys_rstn_i)begin
		mstick_cnt <= 0;
	end
	else if(ustick_cnt == MS_DELAY - 1'b1)begin
		mstick_cnt <= 0;
	end
	else begin
		mstick_cnt <= mstick_cnt + 1'b1;
	end
	end
	
	assign ustick_o = (ustick_cnt >= (US_DELAY >> 1));
	assign mstick_o = (mstick_cnt >= (MS_DELAY >> 1));

endmodule


