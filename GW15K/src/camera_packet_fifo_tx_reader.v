// ============================================================================
// Camera packet FIFO -> RoraLink TX user interface reader
// ----------------------------------------------------------------------------
// Clock domain: RoraLink tx_clk.
// FIFO data format: Q[32:0] = {last, data[31:0]}.
//
// This reader assumes the generated Gowin FIFO is in First-Word Fall-Through
// mode: Q is valid whenever Empty is low, and RdEn advances to the next word.
// It starts a packet only when Rnum shows at least one complete 260-word packet.
// ============================================================================

`timescale 1ns / 1ps

module camera_packet_fifo_tx_reader #(
    parameter [13:0] PACKET_WORDS = 14'd260
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        channel_up,
    input  wire        lane_up,
    input  wire        hard_err,
    input  wire        tx_ready,

    input  wire [32:0] fifo_q,
    input  wire        fifo_empty,
    input  wire [13:0] fifo_rnum,
    output wire        fifo_rd_en,

    output wire [31:0] tx_data,
    output wire [3:0]  tx_strb,
    output wire        tx_valid,
    output wire        tx_last,

    output wire        tx_fire_dbg,
    output reg  [1:0]  tx_state_dbg,
    output reg  [31:0] packet_cnt_dbg,
    output reg  [31:0] word_cnt_dbg,
    output reg         start_seen_dbg,
    output reg         fire_seen_dbg,
    output reg         last_seen_dbg,
    output reg         ready_seen_dbg,
    output reg         channel_seen_dbg,
    output reg         lane_seen_dbg,
    output reg         hard_err_seen_dbg,
    output reg         fifo_empty_seen_dbg,
    output reg         fifo_underflow_seen_dbg
);

localparam [1:0] ST_WAIT = 2'd0;
localparam [1:0] ST_SEND = 2'd1;

reg [1:0] state;

wire link_ok = channel_up && lane_up && !hard_err;
wire start_packet = link_ok && tx_ready && (fifo_rnum >= PACKET_WORDS) && !fifo_empty;

assign tx_data  = fifo_q[31:0];
assign tx_strb  = 4'hF;
assign tx_last  = fifo_q[32];
assign tx_valid = (state == ST_SEND) && link_ok && !fifo_empty;
assign tx_fire_dbg = tx_valid && tx_ready;
assign fifo_rd_en = tx_fire_dbg;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state                    <= ST_WAIT;
        packet_cnt_dbg           <= 32'd0;
        word_cnt_dbg             <= 32'd0;
        start_seen_dbg           <= 1'b0;
        fire_seen_dbg            <= 1'b0;
        last_seen_dbg            <= 1'b0;
        ready_seen_dbg           <= 1'b0;
        channel_seen_dbg         <= 1'b0;
        lane_seen_dbg            <= 1'b0;
        hard_err_seen_dbg        <= 1'b0;
        fifo_empty_seen_dbg      <= 1'b0;
        fifo_underflow_seen_dbg  <= 1'b0;
    end else begin
        if (tx_ready)   ready_seen_dbg   <= 1'b1;
        if (channel_up) channel_seen_dbg <= 1'b1;
        if (lane_up)    lane_seen_dbg    <= 1'b1;
        if (hard_err)   hard_err_seen_dbg <= 1'b1;
        if (fifo_empty) fifo_empty_seen_dbg <= 1'b1;

        if (!link_ok) begin
            state <= ST_WAIT;
        end else begin
            case (state)
                ST_WAIT: begin
                    if (start_packet) begin
                        state <= ST_SEND;
                        start_seen_dbg <= 1'b1;
                    end
                end

                ST_SEND: begin
                    if (fifo_empty) begin
                        fifo_underflow_seen_dbg <= 1'b1;
                        state <= ST_WAIT;
                    end else if (tx_fire_dbg) begin
                        fire_seen_dbg <= 1'b1;
                        word_cnt_dbg  <= word_cnt_dbg + 1'b1;
                        if (tx_last) begin
                            last_seen_dbg  <= 1'b1;
                            packet_cnt_dbg <= packet_cnt_dbg + 1'b1;
                            state <= ST_WAIT;
                        end
                    end
                end

                default: begin
                    state <= ST_WAIT;
                end
            endcase
        end
    end
end

always @(*) begin
    tx_state_dbg = state;
end

endmodule
