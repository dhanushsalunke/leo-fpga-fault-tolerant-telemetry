// tlm_baseline_top.v -- UNPROTECTED reference: a single lane, no redundancy.
// Same stream interface and same in_last -> out_valid latency (2 cycles) as the
// resilient top, so the two designs are directly comparable.
module tlm_baseline_top #(
    parameter FI_EN = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [7:0]  in_data,
    input  wire        in_last,
    output reg         out_valid,
    output reg         out_ok,
    output reg  [7:0]  out_type,
    output reg  [7:0]  out_seq,
    output reg  [15:0] out_sum,
    output reg  [7:0]  out_min,
    output reg  [7:0]  out_max,
    input  wire [95:0] fi_flip,
    input  wire [95:0] fi_smask,
    input  wire [95:0] fi_sval
);
    wire [48:0] v;
    reg s1;
    tlm_lane #(.FI_EN(FI_EN)) u_lane (
        .clk(clk), .rst_n(rst_n), .flush(1'b0),
        .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .vec(v),
        .fi_flip(fi_flip), .fi_smask(fi_smask), .fi_sval(fi_sval));
    always @(posedge clk) begin
        if (!rst_n) begin s1 <= 1'b0; out_valid <= 1'b0; end
        else begin
            s1        <= in_valid & in_last;
            out_valid <= s1;
        end
        if (s1) {out_ok, out_type, out_seq, out_sum, out_min, out_max} <= v;
    end
endmodule
