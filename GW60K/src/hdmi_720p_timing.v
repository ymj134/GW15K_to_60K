`timescale 1ns / 1ps

module hdmi_720p_timing (
    input  wire        pixel_clk,
    input  wire        rst_n,
    output reg         hs,
    output reg         vs,
    output reg         de,
    output reg [11:0]  x,
    output reg [10:0]  y,
    output wire        frame_start
);
    localparam [15:0] H_TOTAL  = 16'd1650;
    localparam [15:0] H_SYNC   = 16'd40;
    localparam [15:0] H_BPORCH = 16'd220;
    localparam [15:0] H_RES    = 16'd1280;
    localparam [15:0] V_TOTAL  = 16'd750;
    localparam [15:0] V_SYNC   = 16'd5;
    localparam [15:0] V_BPORCH = 16'd20;
    localparam [15:0] V_RES    = 16'd720;
    localparam [15:0] H_ACT_ST = H_SYNC + H_BPORCH;
    localparam [15:0] V_ACT_ST = V_SYNC + V_BPORCH;

    reg [15:0] h_cnt;
    reg [15:0] v_cnt;

    assign frame_start = (h_cnt == 16'd0) && (v_cnt == 16'd0);

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 16'd0;
            v_cnt <= 16'd0;
        end else begin
            if (h_cnt == H_TOTAL - 1'b1) begin
                h_cnt <= 16'd0;
                if (v_cnt == V_TOTAL - 1'b1)
                    v_cnt <= 16'd0;
                else
                    v_cnt <= v_cnt + 1'b1;
            end else begin
                h_cnt <= h_cnt + 1'b1;
            end
        end
    end

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            hs <= 1'b0;
            vs <= 1'b0;
            de <= 1'b0;
            x  <= 12'd0;
            y  <= 11'd0;
        end else begin
            hs <= (h_cnt < H_SYNC);
            vs <= (v_cnt < V_SYNC);
            de <= (h_cnt >= H_ACT_ST) && (h_cnt < H_ACT_ST + H_RES) &&
                  (v_cnt >= V_ACT_ST) && (v_cnt < V_ACT_ST + V_RES);
            if ((h_cnt >= H_ACT_ST) && (h_cnt < H_ACT_ST + H_RES))
                x <= h_cnt - H_ACT_ST;
            else
                x <= 12'd0;
            if ((v_cnt >= V_ACT_ST) && (v_cnt < V_ACT_ST + V_RES))
                y <= v_cnt - V_ACT_ST;
            else
                y <= 11'd0;
        end
    end
endmodule


// ============================================================================
// RoraLink segmented video RX -> DDR AXI writer/reader bridge
// B2_fix2 720p policy:
//   - Keep the proven DDR read/HDMI path from A2-1.
//   - Use a lenient depacketizer for B2: header-driven capture, no payload
//     checker in the write path.
//   - Start DDR capture only from SOF: line_id=0, segment_id=0.
//   - Do not let startup hard/frame error or payload checker block DDR output.
// ============================================================================
