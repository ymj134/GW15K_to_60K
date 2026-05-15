/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename            :       I2C_SC130GS_12801024_Config.v
Author              :       CrazyBingo
Date                :       2019-08-03
Version             :       1.0
Description         :       I2C Configure Data of AR0135.
Modification History    :
Date            By          Version         Change Description
===========================================================================
19/08/03        CrazyBingo  1.0             Original
--------------------------------------------------------------------------*/

`timescale 1ns/1ns
module  I2C_ISPCAMERA_4Lanes_Config  //
(
    input       [7:0]   LUT_INDEX,
    output  reg [23:0]  LUT_DATA,
    output      [7:0]   LUT_SIZE
);
assign  LUT_SIZE =1;

//-----------------------------------------------------------------
/////////////////////   Config Data LUT   //////////////////////////    
always@(*)
begin
    case(LUT_INDEX)
0:	LUT_DATA = {16'h0100, 8'h01}; 

		default:LUT_DATA    =   {16'h0000, 8'h00};
    endcase
end

endmodule
