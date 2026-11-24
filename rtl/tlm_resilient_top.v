// tlm_resilient_top.v -- fault-tolerant telemetry processor.
//
//  * 3 identical lanes (TMR, coarse-grained: whole parser/CRC/statistics datapath)
//  * word-level 2-of-3 voter over each lane's 49-bit result vector at frame end
//  * per-lane mismatch history -> lane isolation (persistent-fault handling)
//  * isolated lane keeps running as a "shadow"; after PROB_N consecutive frames
//    that match the accepted output it is re-integrated (transient-burst recovery)
//  * degradation: 3 healthy = TMR, 2 healthy = DMR (detect, drop on disagreement),
//    1 healthy = SAFE (single lane, output flagged `degraded`, unverified)
//  * stream watchdog: stalled mid-frame -> flush all lanes
//
// "Lane isolation / lane flush" is a CONTROLLER-LEVEL ABSTRACTION of what a
// partial-reconfiguration / scrubbing flow would do.  No dynamic partial
// reconfiguration is implemented or claimed.  The voter, controller and watchdog
// are NOT redundant (single points of failure; see docs/FAULT_MODEL_AND_LIMITATIONS.md).
module tlm_resilient_top #(
    parameter FI_EN      = 0,
    parameter ISO_EN     = 1,    // 0 = vote only, never isolate (ablation)
    parameter ISO_THRESH = 6,    // mismatches within the last 16 frames that isolate a lane
    parameter PROB_N     = 8,    // clean frames needed to re-integrate a lane
    parameter WD_LIMIT   = 32    // idle cycles inside a frame before the watchdog fires
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [7:0]  in_data,
    input  wire        in_last,
    output reg         out_valid,
    output reg         out_ok,        // 0 = frame rejected (link error or uncorrectable)
    output reg  [7:0]  out_type,
    output reg  [7:0]  out_seq,
    output reg  [15:0] out_sum,
    output reg  [7:0]  out_min,
    output reg  [7:0]  out_max,
    output reg         st_fault,      // a lane disagreed on this frame (detected)
    output reg         st_uncorr,     // DMR disagreement: frame dropped
    output reg         st_degraded,   // SAFE mode: unverified single-lane output
    output reg  [2:0]  iso,           // isolated-lane mask
    output reg         wd_event,      // watchdog fired
    output reg  [15:0] det_count,
    input  wire [3*96-1:0] fi_flip,
    input  wire [3*96-1:0] fi_smask,
    input  wire [3*96-1:0] fi_sval
);
    // ---------------- lanes ----------------
    wire [2:0]  lane_flush;
    wire [48:0] v0, v1, v2;
    tlm_lane #(.FI_EN(FI_EN)) u_l0 (.clk(clk), .rst_n(rst_n), .flush(lane_flush[0]),
        .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .vec(v0),
        .fi_flip(fi_flip[0*96 +: 96]), .fi_smask(fi_smask[0*96 +: 96]), .fi_sval(fi_sval[0*96 +: 96]));
    tlm_lane #(.FI_EN(FI_EN)) u_l1 (.clk(clk), .rst_n(rst_n), .flush(lane_flush[1]),
        .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .vec(v1),
        .fi_flip(fi_flip[1*96 +: 96]), .fi_smask(fi_smask[1*96 +: 96]), .fi_sval(fi_sval[1*96 +: 96]));
    tlm_lane #(.FI_EN(FI_EN)) u_l2 (.clk(clk), .rst_n(rst_n), .flush(lane_flush[2]),
        .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .vec(v2),
        .fi_flip(fi_flip[2*96 +: 96]), .fi_smask(fi_smask[2*96 +: 96]), .fi_sval(fi_sval[2*96 +: 96]));

    // ---------------- stream watchdog ----------------
    reg       in_pkt;
    reg [7:0] wd_cnt;
    wire wd_fire = in_pkt && !in_valid && (wd_cnt == WD_LIMIT - 1);
    always @(posedge clk) begin
        if (!rst_n) begin in_pkt <= 1'b0; wd_cnt <= 8'd0; wd_event <= 1'b0; end
        else begin
            wd_event <= wd_fire;
            if (in_valid) begin
                wd_cnt <= 8'd0;
                in_pkt <= !in_last;
            end else if (wd_fire) begin
                in_pkt <= 1'b0; wd_cnt <= 8'd0;
            end else if (in_pkt) wd_cnt <= wd_cnt + 8'd1;
        end
    end

    // ---------------- voter + recovery controller ----------------
    reg s1;
    // DELIVERY: word-level 2-of-3 vote.  A result is trusted only if two lanes produce the IDENTICAL
    // 49-bit vector.  (A bitwise vote can "repair" two coincident faults into a plausible but wrong value;
    // an earlier bitwise-only version showed exactly that in the campaign.  With no agreeing pair the
    // frame is rejected and flagged uncorrectable.)
    wire eq01 = (v0 == v1), eq02 = (v0 == v2), eq12 = (v1 == v2);
    wire all_eq   = eq01 & eq02;
    wire any_pair = eq01 | eq02 | eq12;
    wire [48:0] maj = (eq01 | eq02) ? v0 : v1;
    // DIAGNOSIS: per-lane deviation from the BITWISE majority.  Used only to attribute blame (history /
    // isolation); it localises a lane whose faults are confined to a few bits even when two lanes are bad.
    wire [48:0] bmaj = (v0 & v1) | (v0 & v2) | (v1 & v2);
    wire [2:0]  mm  = {v2 != bmaj, v1 != bmaj, v0 != bmaj};
    wire [2:0]  healthy = ~iso;
    wire [1:0]  nh = {1'b0, healthy[0]} + {1'b0, healthy[1]} + {1'b0, healthy[2]};

    reg [15:0] hist0, hist1, hist2;
    reg [4:0] prob0, prob1, prob2;

    wire [48:0] va = healthy[0] ? v0 : v1;                 // first healthy lane
    wire [48:0] vb = healthy[2] ? v2 : v1;                 // last  healthy lane
    reg  [48:0] sel;                                       // value delivered for this frame
    reg         uncorr, degr, flt;
    always @* begin
        uncorr = 1'b0; degr = 1'b0; flt = 1'b0; sel = maj;
        case (nh)
            2'd3: begin sel = maj; flt = ~all_eq; uncorr = ~any_pair; end
            2'd2: begin sel = va; flt = (va != vb); uncorr = (va != vb); end
            default: begin sel = healthy[0] ? v0 : (healthy[1] ? v1 : v2); degr = 1'b1; end
        endcase
    end

    function [4:0] pop16;
        input [15:0] h;
        integer k;
        begin pop16 = 5'd0; for (k = 0; k < 16; k = k + 1) pop16 = pop16 + {4'b0000, h[k]}; end
    endfunction

    // blame is attributable with three votes (TMR) or when a DMR tie is broken by the shadow lane
    wire blame_en = (nh == 2'd3) || ((nh == 2'd2) && uncorr);
    wire [15:0] h0n = {hist0[14:0], healthy[0] & blame_en & mm[0]};
    wire [15:0] h1n = {hist1[14:0], healthy[1] & blame_en & mm[1]};
    wire [15:0] h2n = {hist2[14:0], healthy[2] & blame_en & mm[2]};
    wire [4:0] p0 = pop16(h0n), p1 = pop16(h1n), p2 = pop16(h2n);
    wire c0 = healthy[0] && (p0 >= ISO_THRESH);
    wire c1 = healthy[1] && (p1 >= ISO_THRESH);
    wire c2 = healthy[2] && (p2 >= ISO_THRESH);
    // isolate at most one lane per frame: the worst offender (ties -> lowest index)
    wire pick0 = c0 && (!c1 || p0 >= p1) && (!c2 || p0 >= p2);
    wire pick1 = c1 && !pick0 && (!c2 || p1 >= p2);
    wire pick2 = c2 && !pick0 && !pick1;
    wire can_iso = (ISO_EN != 0) && (nh >= 2'd2);
    wire [2:0] iso_new = (can_iso && s1) ? {pick2, pick1, pick0} : 3'b000;

    // probation reference: the delivered value (in SAFE mode this is the single healthy lane,
    // so PROB_N consecutive agreeing frames are required before an isolated lane rejoins)
    wire ref_ok = !uncorr;
    wire [2:0] match = {v2 == sel, v1 == sel, v0 == sel};
    wire [2:0] reint;
    assign reint[0] = iso[0] && ref_ok && match[0] && (prob0 == PROB_N - 1);
    assign reint[1] = iso[1] && ref_ok && match[1] && (prob1 == PROB_N - 1);
    assign reint[2] = iso[2] && ref_ok && match[2] && (prob2 == PROB_N - 1);

    // lanes are flushed on watchdog timeout and when newly isolated (reconfiguration abstraction)
    assign lane_flush = {3{wd_event}} | iso_new;

    always @(posedge clk) begin
        if (!rst_n) begin
            s1 <= 1'b0; out_valid <= 1'b0; iso <= 3'b000;
            hist0 <= 16'd0; hist1 <= 16'd0; hist2 <= 16'd0;
            prob0 <= 5'd0; prob1 <= 5'd0; prob2 <= 5'd0;
            st_fault <= 1'b0; st_uncorr <= 1'b0; st_degraded <= 1'b0;
            det_count <= 16'd0;
        end else begin
            s1 <= in_valid & in_last;
            out_valid <= s1;
            st_fault <= 1'b0; st_uncorr <= 1'b0; st_degraded <= 1'b0;
            if (s1) begin
                {out_ok, out_type, out_seq, out_sum, out_min, out_max} <= sel;
                if (uncorr) out_ok <= 1'b0;
                st_fault <= flt; st_uncorr <= uncorr; st_degraded <= degr;
                if (flt) det_count <= det_count + 16'd1;
                hist0 <= iso_new[0] ? 16'd0 : h0n;
                hist1 <= iso_new[1] ? 16'd0 : h1n;
                hist2 <= iso_new[2] ? 16'd0 : h2n;
                prob0 <= (iso_new[0] || reint[0]) ? 5'd0 : (iso[0] && ref_ok && match[0]) ? prob0 + 5'd1 : 5'd0;
                prob1 <= (iso_new[1] || reint[1]) ? 5'd0 : (iso[1] && ref_ok && match[1]) ? prob1 + 5'd1 : 5'd0;
                prob2 <= (iso_new[2] || reint[2]) ? 5'd0 : (iso[2] && ref_ok && match[2]) ? prob2 + 5'd1 : 5'd0;
                iso <= (iso | iso_new) & ~reint;
            end
        end
    end
endmodule
