// tlm_lane.v -- one telemetry-processing lane: packet parser + CRC-16 check +
// payload statistics (sum/min/max).  All architectural state lives in ONE packed
// register vector `st` so that the verification-only fault-injection hooks
// (FI_EN=1) can flip or stick any flip-flop.  With FI_EN=0 the hooks are not
// elaborated and the lane is plain synchronous logic.
//
// Packet (bytes):  A5 5A | TYPE | SEQ | LEN(1..32) | PAYLOAD[LEN] | CRC_HI | CRC_LO
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over TYPE,SEQ,LEN,PAYLOAD.
// `in_last` marks the final byte of a link-layer frame (e.g. SpaceWire-style EOP).
// Per-frame state is fully re-initialised by the next frame start, and the FSM
// returns to IDLE at every in_last, so a transient upset is confined to one frame.
module tlm_lane #(
    parameter FI_EN = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        flush,        // synchronous re-initialisation of the lane
    input  wire        in_valid,
    input  wire [7:0]  in_data,
    input  wire        in_last,
    output wire [48:0] vec,          // {ok,type,seq,sum,min,max}: valid 1 cycle after in_last
    // verification-only fault injection (ignored when FI_EN=0)
    input  wire [95:0] fi_flip,      // XOR mask applied in every cycle it is non-zero
    input  wire [95:0] fi_smask,     // stuck-at mask (level)
    input  wire [95:0] fi_sval       // stuck-at value
);
    localparam STW = 89;
    localparam S_IDLE=4'd0, S_SYNC2=4'd1, S_TYPE=4'd2, S_SEQ=4'd3, S_LEN=4'd4,
               S_PAY=4'd5,  S_CRC1=4'd6,  S_CRC2=4'd7, S_DROP=4'd8;
    // field positions inside st
    localparam P_ST=0, P_CRC=4, P_CRCHI=20, P_TYPE=28, P_SEQ=36, P_LEN=44,
               P_CNT=50, P_SUM=56, P_MN=72, P_MX=80, P_OK=88;
    localparam [STW-1:0] INIT = {1'b0, 8'h00, 8'hFF, 16'h0000, 6'd0, 6'd0, 8'h00, 8'h00,
                                 8'h00, 16'h0000, 4'd0};

    function [15:0] crc16;
        input [15:0] c;
        input [7:0]  d;
        integer i;
        reg [15:0] r;
        begin
            r = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                r = r[15] ? ((r << 1) ^ 16'h1021) : (r << 1);
            crc16 = r;
        end
    endfunction

    reg [STW-1:0] st, nx;
    wire [3:0]  f_state = st[P_ST +: 4];
    wire [15:0] f_crc   = st[P_CRC +: 16];
    wire [5:0]  f_len   = st[P_LEN +: 6];
    wire [5:0]  f_cnt   = st[P_CNT +: 6];
    wire [15:0] f_sum   = st[P_SUM +: 16];
    wire [7:0]  f_mn    = st[P_MN +: 8];
    wire [7:0]  f_mx    = st[P_MX +: 8];

    always @* begin
        nx = st;
        if (in_valid) begin
            case (f_state)
                S_IDLE:  if (in_data == 8'hA5) nx[P_ST +: 4] = S_SYNC2;
                S_SYNC2: begin
                    if (in_data == 8'h5A) begin
                        nx[P_ST +: 4]   = S_TYPE;
                        nx[P_CRC +: 16] = 16'hFFFF;
                        nx[P_SUM +: 16] = 16'd0;
                        nx[P_MN +: 8]   = 8'hFF;
                        nx[P_MX +: 8]   = 8'h00;
                        nx[P_CNT +: 6]  = 6'd0;
                    end else nx[P_ST +: 4] = S_IDLE;
                end
                S_TYPE: begin
                    nx[P_TYPE +: 8] = in_data;
                    nx[P_CRC +: 16] = crc16(f_crc, in_data);
                    nx[P_ST +: 4]   = S_SEQ;
                end
                S_SEQ: begin
                    nx[P_SEQ +: 8]  = in_data;
                    nx[P_CRC +: 16] = crc16(f_crc, in_data);
                    nx[P_ST +: 4]   = S_LEN;
                end
                S_LEN: begin
                    nx[P_LEN +: 6]  = in_data[5:0];
                    nx[P_CRC +: 16] = crc16(f_crc, in_data);
                    nx[P_ST +: 4]   = (in_data == 8'd0 || in_data > 8'd32) ? S_DROP : S_PAY;
                end
                S_PAY: begin
                    nx[P_SUM +: 16] = f_sum + {8'h00, in_data};
                    if (in_data < f_mn) nx[P_MN +: 8] = in_data;
                    if (in_data > f_mx) nx[P_MX +: 8] = in_data;
                    nx[P_CRC +: 16] = crc16(f_crc, in_data);
                    nx[P_CNT +: 6]  = f_cnt + 6'd1;
                    if (f_cnt + 6'd1 == f_len) nx[P_ST +: 4] = S_CRC1;
                end
                S_CRC1: begin
                    nx[P_CRCHI +: 8] = in_data;
                    nx[P_ST +: 4]    = S_CRC2;
                end
                S_CRC2:  nx[P_ST +: 4] = S_IDLE;
                default: nx[P_ST +: 4] = S_DROP;     // S_DROP and unreachable codes
            endcase
            if (in_last) begin
                nx[P_OK] = (f_state == S_CRC2) && ({st[P_CRCHI +: 8], in_data} == f_crc);
                nx[P_ST +: 4] = S_IDLE;
            end
        end
    end

    generate
        if (FI_EN) begin : g_fi
            wire [STW-1:0] m = fi_smask[STW-1:0];
            wire [STW-1:0] v = fi_sval[STW-1:0];
            always @(posedge clk) begin
                if (!rst_n || flush) st <= (INIT & ~m) | (v & m);
                else                 st <= ((nx ^ fi_flip[STW-1:0]) & ~m) | (v & m);
            end
        end else begin : g_nofi
            always @(posedge clk) begin
                if (!rst_n || flush) st <= INIT;
                else                 st <= nx;
            end
        end
    endgenerate

    assign vec = {st[P_OK], st[P_TYPE +: 8], st[P_SEQ +: 8], st[P_SUM +: 16],
                  st[P_MN +: 8], st[P_MX +: 8]};
endmodule
