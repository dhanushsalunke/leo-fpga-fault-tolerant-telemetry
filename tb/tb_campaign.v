// tb_campaign.v -- data-driven testbench for fault-injection campaigns.
// Stimulus and fault plan are read from .mem files produced by model/tlm_model.py.
// The testbench only *logs*; scoring against the golden model is done in
// scripts/analyze.py.  Build with -DRESILIENT for the 3-lane design, otherwise the
// single-lane baseline.  Plusargs: +pfx=<mem prefix> +ncyc=<n> +nev=<n> +log=<file>
`timescale 1ns/1ps
`ifndef ISO_EN
`define ISO_EN 1
`endif
module tb_campaign;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg [11:0]  stim [0:199999];
    reg [31:0]  ev_c [0:99999];
    reg [3:0]   ev_l [0:99999];
    reg [95:0]  ev_f [0:99999];
    reg [95:0]  ev_m [0:99999];
    reg [95:0]  ev_v [0:99999];

    reg         in_valid = 0, in_last = 0;
    reg [7:0]   in_data = 0;
    reg [3*96-1:0] fi_flip = 0, fi_smask = 0, fi_sval = 0;

    integer ncyc, nev, ptr, cyc, lg, i;
    reg [1023:0] pfx_s, log_s, fn;
    reg [1:0] state_seen_dummy;

    wire        out_valid, out_ok;
    wire [7:0]  out_type, out_seq, out_min, out_max;
    wire [15:0] out_sum;

`ifdef RESILIENT
    wire st_fault, st_uncorr, st_degraded, wd_event;
    wire [2:0] iso;
    wire [15:0] det_count;
    tlm_resilient_top #(.FI_EN(1), .ISO_EN(`ISO_EN)) dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid), .in_data(in_data), .in_last(in_last),
        .out_valid(out_valid), .out_ok(out_ok), .out_type(out_type), .out_seq(out_seq),
        .out_sum(out_sum), .out_min(out_min), .out_max(out_max),
        .st_fault(st_fault), .st_uncorr(st_uncorr), .st_degraded(st_degraded),
        .iso(iso), .wd_event(wd_event), .det_count(det_count),
        .fi_flip(fi_flip), .fi_smask(fi_smask), .fi_sval(fi_sval));
`else
    tlm_baseline_top #(.FI_EN(1)) dut (
        .clk(clk), .rst_n(rst_n), .in_valid(in_valid), .in_data(in_data), .in_last(in_last),
        .out_valid(out_valid), .out_ok(out_ok), .out_type(out_type), .out_seq(out_seq),
        .out_sum(out_sum), .out_min(out_min), .out_max(out_max),
        .fi_flip(fi_flip[95:0]), .fi_smask(fi_smask[95:0]), .fi_sval(fi_sval[95:0]));
`endif

    // FSM-state coverage per lane (hierarchical probes)
    reg [8:0] seen0 = 0, seen1 = 0, seen2 = 0;
`ifdef RESILIENT
    always @(posedge clk) begin
        seen0[dut.u_l0.f_state] <= 1; seen1[dut.u_l1.f_state] <= 1; seen2[dut.u_l2.f_state] <= 1;
    end
`else
    always @(posedge clk) seen0[dut.u_lane.f_state] <= 1;
`endif

    initial begin
        if (!$value$plusargs("pfx=%s", pfx_s)) begin $display("missing +pfx"); $finish; end
        if (!$value$plusargs("log=%s", log_s)) begin $display("missing +log"); $finish; end
        if (!$value$plusargs("ncyc=%d", ncyc)) ncyc = 0;
        if (!$value$plusargs("nev=%d", nev)) nev = 0;
        $sformat(fn, "%0s_stim.mem", pfx_s); $readmemh(fn, stim);
        $sformat(fn, "%0s_ev.mem",   pfx_s); $readmemh(fn, ev_c);
        $sformat(fn, "%0s_evl.mem",  pfx_s); $readmemh(fn, ev_l);
        $sformat(fn, "%0s_evf.mem",  pfx_s); $readmemh(fn, ev_f);
        $sformat(fn, "%0s_evm.mem",  pfx_s); $readmemh(fn, ev_m);
        $sformat(fn, "%0s_evv.mem",  pfx_s); $readmemh(fn, ev_v);
        lg = $fopen(log_s, "w");
        ptr = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        for (cyc = 0; cyc < ncyc; cyc = cyc + 1) begin
            // apply inputs and faults for cycle `cyc` (sampled by the next posedge)
            @(negedge clk);
            in_valid = stim[cyc][9]; in_last = stim[cyc][8]; in_data = stim[cyc][7:0];
            fi_flip = 0;
            while (ptr < nev && ev_c[ptr] == cyc) begin
                fi_flip[ev_l[ptr]*96 +: 96] = fi_flip[ev_l[ptr]*96 +: 96] ^ ev_f[ptr];
                if (ev_m[ptr] != 0) begin    // stuck-at: persistent from this cycle on
                    fi_sval[ev_l[ptr]*96 +: 96] = (fi_sval[ev_l[ptr]*96 +: 96] & ~ev_m[ptr]) | (ev_v[ptr] & ev_m[ptr]);
                    fi_smask[ev_l[ptr]*96 +: 96] = fi_smask[ev_l[ptr]*96 +: 96] | ev_m[ptr];
                end
                ptr = ptr + 1;
            end
        end
        @(negedge clk); in_valid = 0; in_last = 0;
        repeat (6) @(posedge clk);
`ifdef RESILIENT
        $fdisplay(lg, "C %0d %0d %0d", seen0, seen1, seen2);
`else
        $fdisplay(lg, "C %0d 0 0", seen0);
`endif
        $fclose(lg);
        $finish;
    end

    // per-frame log: lane vectors at the sample cycle, and the DUT output record
    reg s1_mon;
    always @(posedge clk) begin
        s1_mon <= in_valid & in_last;
        if (s1_mon) begin
`ifdef RESILIENT
            $fdisplay(lg, "L %0d %h %h %h", cyc-2, dut.v0, dut.v1, dut.v2);
`else
            $fdisplay(lg, "L %0d %h 0 0", cyc-2, dut.v);
`endif
        end
        if (out_valid) begin
`ifdef RESILIENT
            $fdisplay(lg, "O %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                cyc-3, out_ok, out_type, out_seq, out_sum, out_min, out_max,
                st_fault, st_uncorr, st_degraded, iso, wd_event, det_count);
`else
            $fdisplay(lg, "O %0d %0d %0d %0d %0d %0d %0d 0 0 0 0 0 0",
                cyc-3, out_ok, out_type, out_seq, out_sum, out_min, out_max);
`endif
        end
    end
endmodule
