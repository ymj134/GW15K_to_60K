// ============================================================================
// Small generic asynchronous FIFO with Gray-coded pointer synchronization
// ----------------------------------------------------------------------------
// Read mode is registered, not FWFT:
//   assert rd_en for one rd_clk cycle when !empty;
//   rd_data is valid after the read clock edge and should be sampled one cycle
//   later by the reader FSM.
// ============================================================================

`timescale 1ns / 1ps

module async_fifo_gray #(
    parameter integer DATA_WIDTH = 64,
    parameter integer ADDR_WIDTH = 11       // depth = 2**ADDR_WIDTH
)(
    input  wire                    rst,

    input  wire                    wr_clk,
    input  wire                    wr_en,
    input  wire [DATA_WIDTH-1:0]   wr_data,
    output wire                    full,
    output wire [ADDR_WIDTH:0]     wr_count,

    input  wire                    rd_clk,
    input  wire                    rd_en,
    output reg  [DATA_WIDTH-1:0]   rd_data,
    output wire                    empty,
    output wire [ADDR_WIDTH:0]     rd_count
);

localparam integer DEPTH = (1 << ADDR_WIDTH);

reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

reg [ADDR_WIDTH:0] wr_bin;
reg [ADDR_WIDTH:0] wr_gray;
reg [ADDR_WIDTH:0] rd_bin;
reg [ADDR_WIDTH:0] rd_gray;

reg [ADDR_WIDTH:0] rd_gray_w1, rd_gray_w2;
reg [ADDR_WIDTH:0] wr_gray_r1, wr_gray_r2;

wire wr_do = wr_en && !full;
wire rd_do = rd_en && !empty;

wire [ADDR_WIDTH:0] wr_bin_next  = wr_bin + {{ADDR_WIDTH{1'b0}}, wr_do};
wire [ADDR_WIDTH:0] rd_bin_next  = rd_bin + {{ADDR_WIDTH{1'b0}}, rd_do};
wire [ADDR_WIDTH:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
wire [ADDR_WIDTH:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

assign full  = (wr_gray_next == {~rd_gray_w2[ADDR_WIDTH:ADDR_WIDTH-1], rd_gray_w2[ADDR_WIDTH-2:0]});
assign empty = (rd_gray == wr_gray_r2);

function [ADDR_WIDTH:0] gray_to_bin;
    input [ADDR_WIDTH:0] gray;
    integer i;
    begin
        gray_to_bin[ADDR_WIDTH] = gray[ADDR_WIDTH];
        for (i = ADDR_WIDTH-1; i >= 0; i = i - 1)
            gray_to_bin[i] = gray_to_bin[i+1] ^ gray[i];
    end
endfunction

wire [ADDR_WIDTH:0] rd_bin_sync_w = gray_to_bin(rd_gray_w2);
wire [ADDR_WIDTH:0] wr_bin_sync_r = gray_to_bin(wr_gray_r2);

assign wr_count = wr_bin - rd_bin_sync_w;
assign rd_count = wr_bin_sync_r - rd_bin;

always @(posedge wr_clk or posedge rst) begin
    if (rst) begin
        wr_bin     <= {ADDR_WIDTH+1{1'b0}};
        wr_gray    <= {ADDR_WIDTH+1{1'b0}};
        rd_gray_w1 <= {ADDR_WIDTH+1{1'b0}};
        rd_gray_w2 <= {ADDR_WIDTH+1{1'b0}};
    end else begin
        rd_gray_w1 <= rd_gray;
        rd_gray_w2 <= rd_gray_w1;

        if (wr_do) begin
            mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
        end
    end
end

always @(posedge rd_clk or posedge rst) begin
    if (rst) begin
        rd_bin     <= {ADDR_WIDTH+1{1'b0}};
        rd_gray    <= {ADDR_WIDTH+1{1'b0}};
        wr_gray_r1 <= {ADDR_WIDTH+1{1'b0}};
        wr_gray_r2 <= {ADDR_WIDTH+1{1'b0}};
        rd_data    <= {DATA_WIDTH{1'b0}};
    end else begin
        wr_gray_r1 <= wr_gray;
        wr_gray_r2 <= wr_gray_r1;

        if (rd_do) begin
            rd_data <= mem[rd_bin[ADDR_WIDTH-1:0]];
            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
        end
    end
end

endmodule
