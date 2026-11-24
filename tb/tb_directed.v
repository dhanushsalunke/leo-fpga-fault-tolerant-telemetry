// tb_directed.v -- self-checking directed tests.
// Three DUTs see the same byte stream:  b = baseline (1 lane),  r = resilient,
// n = resilient with lane isolation disabled (ISO_EN=0, ablation).
// Every check prints PASS/FAIL; the run ends with "ALL DIRECTED TESTS PASSED" or a failure count.
`timescale 1ns/1ps
module tb_directed;
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;
    integer cc = 0;
    always @(posedge clk) cc <= cc + 1;

    reg        in_valid = 0, in_last = 0;
    reg [7:0]  in_data = 0;
    reg [95:0]    fb_flip = 0, fb_smask = 0, fb_sval = 0;
    reg [287:0]   fr_flip = 0, fr_smask = 0, fr_sval = 0, fn_flip = 0, fn_smask = 0, fn_sval = 0;

    wire b_v, b_ok; wire [7:0] b_type, b_seq, b_min, b_max; wire [15:0] b_sum;
    wire r_v, r_ok; wire [7:0] r_type, r_seq, r_min, r_max; wire [15:0] r_sum;
    wire r_flt, r_unc, r_deg, r_wd; wire [2:0] r_iso; wire [15:0] r_dc;
    wire n_v, n_ok; wire [7:0] n_type, n_seq, n_min, n_max; wire [15:0] n_sum;
    wire n_flt, n_unc, n_deg, n_wd; wire [2:0] n_iso; wire [15:0] n_dc;

    tlm_baseline_top #(.FI_EN(1)) db (.clk(clk), .rst_n(rst_n), .in_valid(in_valid), .in_data(in_data),
        .in_last(in_last), .out_valid(b_v), .out_ok(b_ok), .out_type(b_type), .out_seq(b_seq),
        .out_sum(b_sum), .out_min(b_min), .out_max(b_max),
        .fi_flip(fb_flip), .fi_smask(fb_smask), .fi_sval(fb_sval));
    tlm_resilient_top #(.FI_EN(1), .ISO_EN(1)) dr (.clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .in_data(in_data), .in_last(in_last), .out_valid(r_v), .out_ok(r_ok), .out_type(r_type),
        .out_seq(r_seq), .out_sum(r_sum), .out_min(r_min), .out_max(r_max), .st_fault(r_flt),
        .st_uncorr(r_unc), .st_degraded(r_deg), .iso(r_iso), .wd_event(r_wd), .det_count(r_dc),
        .fi_flip(fr_flip), .fi_smask(fr_smask), .fi_sval(fr_sval));
    tlm_resilient_top #(.FI_EN(1), .ISO_EN(0)) dn (.clk(clk), .rst_n(rst_n), .in_valid(in_valid),
        .in_data(in_data), .in_last(in_last), .out_valid(n_v), .out_ok(n_ok), .out_type(n_type),
        .out_seq(n_seq), .out_sum(n_sum), .out_min(n_min), .out_max(n_max), .st_fault(n_flt),
        .st_uncorr(n_unc), .st_degraded(n_deg), .iso(n_iso), .wd_event(n_wd), .det_count(n_dc),
        .fi_flip(fn_flip), .fi_smask(fn_smask), .fi_sval(fn_sval));

    // ---------------- output capture ----------------
    integer b_n = 0, r_n = 0, n_n = 0, b_lat = 0, r_lat = 0, t_last = 0, wd_seen = 0;
    reg        b_lok, r_lok, n_lok, r_lflt, r_lunc, r_ldeg, n_lunc;
    reg [7:0]  b_ltype, b_lseq, b_lmin, b_lmax, r_ltype, r_lseq, r_lmin, r_lmax, n_ltype;
    reg [15:0] b_lsum, r_lsum;
    always @(posedge clk) begin
        if (b_v) begin b_n <= b_n + 1; b_lat <= cc - t_last; b_lok <= b_ok; b_ltype <= b_type; b_lseq <= b_seq;
                       b_lsum <= b_sum; b_lmin <= b_min; b_lmax <= b_max; end
        if (r_v) begin r_n <= r_n + 1; r_lat <= cc - t_last; r_lok <= r_ok; r_ltype <= r_type; r_lseq <= r_seq;
                       r_lsum <= r_sum; r_lmin <= r_min; r_lmax <= r_max; r_lflt <= r_flt; r_lunc <= r_unc; r_ldeg <= r_deg; end
        if (n_v) begin n_n <= n_n + 1; n_lok <= n_ok; n_ltype <= n_type; n_lunc <= n_unc; end
        if (r_wd) wd_seen <= wd_seen + 1;
    end

    // ---------------- bookkeeping ----------------
    integer checks = 0, errors = 0;
    task chk(input cond, input [8*100-1:0] msg);
        begin
            checks = checks + 1;
            if (!cond) begin errors = errors + 1; $display("FAIL: %0s", msg); end
            else $display("PASS: %0s", msg);
        end
    endtask

    // ---------------- frame construction / driving ----------------
    reg [7:0] fr [0:63];
    integer flen;
    reg       e_ok; reg [7:0] e_type, e_seq, e_min, e_max; reg [15:0] e_sum;
    reg fork_flip1 = 0;
    integer corrupt_idx = -1, inj_idx = -1, inj_lane = 0, partial = 0;
    reg [95:0] inj_mask = 0;

    function [15:0] crc_byte; input [15:0] c; input [7:0] d; integer i; reg [15:0] r;
        begin r = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1) r = r[15] ? ((r << 1) ^ 16'h1021) : (r << 1);
            crc_byte = r; end
    endfunction

    task mk(input [7:0] ty, input [7:0] sq, input integer len, input [7:0] pat);
        integer i; reg [15:0] c; reg [7:0] d;
        begin
            fr[0] = 8'hA5; fr[1] = 8'h5A; fr[2] = ty; fr[3] = sq; fr[4] = len;
            c = 16'hFFFF; c = crc_byte(c, ty); c = crc_byte(c, sq); c = crc_byte(c, len[7:0]);
            e_sum = 0; e_min = 8'hFF; e_max = 8'h00;
            for (i = 0; i < len; i = i + 1) begin
                d = (pat + 8'd37 * i[7:0]) ^ (i[7:0] * i[7:0]) ^ 8'h40;
                fr[5 + i] = d; c = crc_byte(c, d); e_sum = e_sum + d;
                if (d < e_min) e_min = d; if (d > e_max) e_max = d;
            end
            fr[5 + len] = c[15:8]; fr[6 + len] = c[7:0]; flen = 7 + len;
            e_ok = (len >= 1 && len <= 32); e_type = ty; e_seq = sq;
        end
    endtask

    task set_flip(input integer lane, input [95:0] m);
        begin
            if (lane == 0) fb_flip = m;
            fr_flip[lane*96 +: 96] = m; fn_flip[lane*96 +: 96] = m;
        end
    endtask
    task set_stuck(input integer lane, input [95:0] m, input [95:0] v);
        begin
            if (lane == 0) begin fb_smask = m; fb_sval = v; end
            fr_smask[lane*96 +: 96] = m; fr_sval[lane*96 +: 96] = v;
            fn_smask[lane*96 +: 96] = m; fn_sval[lane*96 +: 96] = v;
        end
    endtask

    task send;   // drives one frame (with optional corruption / one injection / truncation), then idles
        integer i, nsend;
        begin
            if (corrupt_idx >= 0) fr[corrupt_idx] = fr[corrupt_idx] ^ 8'h10;
            nsend = (partial > 0) ? partial : flen;
            for (i = 0; i < nsend; i = i + 1) begin
                @(negedge clk);
                in_valid = 1; in_data = fr[i]; in_last = (partial == 0) && (i == flen - 1);
                if (in_last) t_last = cc;
                if (i == inj_idx) begin
                    set_flip(inj_lane, inj_mask);
                    if (fork_flip1) fr_flip[1*96 +: 96] = (96'd1 << (56 + 3));   // second lane, different bit
                end
                @(posedge clk); #1;
                if (i == inj_idx) begin set_flip(inj_lane, 0); fork_flip1 = 0; end
            end
            @(negedge clk); in_valid = 0; in_last = 0;
            repeat (6) @(posedge clk);
            corrupt_idx = -1; inj_idx = -1; partial = 0;
        end
    endtask

    task do_reset;
        begin
            fb_smask = 0; fr_smask = 0; fn_smask = 0; fb_flip = 0; fr_flip = 0; fn_flip = 0;
            @(negedge clk); rst_n = 0; repeat (3) @(posedge clk); @(negedge clk); rst_n = 1; repeat (2) @(posedge clk);
        end
    endtask

    function r_good; input dummy; begin
        r_good = r_lok && r_lsum == e_sum && r_lmin == e_min && r_lmax == e_max && r_ltype == e_type && r_lseq == e_seq; end
    endfunction
    function b_good; input dummy; begin
        b_good = b_lok && b_lsum == e_sum && b_lmin == e_min && b_lmax == e_max && b_ltype == e_type && b_lseq == e_seq; end
    endfunction

    integer k, bad, dc0;
    localparam [95:0] BIT_OK = 96'd1 << 88, BIT_SUM10 = 96'd1 << (56 + 10), BIT_TYPE0 = 96'd1 << 28,
                      BIT_SEQ0 = 96'd1 << 36;

    initial begin
        repeat (4) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);

        // ---- T1: valid frame accepted, correct fields, 2-cycle latency (both designs) ----
        mk(8'h10, 8'd7, 12, 8'h21); send;
        chk(b_good(0) && r_good(0), "T1 valid frame accepted with correct fields (baseline, resilient)");
        chk(b_lat == 2 && r_lat == 2, "T1 in_last->out_valid latency is 2 cycles for both designs");
        chk(!r_lflt && !r_lunc && !r_ldeg, "T1 no fault flags on clean frame");

        // ---- T2: link-layer corruption (payload bit error) rejected by CRC ----
        mk(8'h02, 8'd8, 10, 8'h33); corrupt_idx = 8; send;
        chk(!b_lok && !r_lok && r_n == b_n, "T2 corrupted payload rejected (CRC)");
        mk(8'h02, 8'd9, 10, 8'h35); corrupt_idx = 15; send;   // corrupt CRC high byte
        chk(!b_lok && !r_lok, "T2b corrupted CRC high byte rejected");
        mk(8'h02, 8'd9, 10, 8'h35); corrupt_idx = 16; send;   // corrupt CRC low byte
        chk(!b_lok && !r_lok, "T2b2 corrupted CRC low byte rejected");
        mk(8'h02, 8'd10, 10, 8'h35); corrupt_idx = 0; send;   // corrupt sync byte
        chk(!b_lok && !r_lok, "T2c corrupted sync byte -> frame not recognised, rejected");

        // ---- T3: length validation ----
        mk(8'h01, 8'd1, 0, 8'h50); send;                       // well-formed frame (valid CRC) but LEN=0
        chk(!r_lok && !b_lok, "T3 LEN=0 (valid CRC) rejected");
        mk(8'h01, 8'd2, 33, 8'h50); send;                      // well-formed frame (valid CRC) but LEN=33
        chk(!r_lok && !b_lok, "T3b LEN=33 (valid CRC) rejected");
        mk(8'h01, 8'd3, 32, 8'h50); send;
        chk(r_good(0) && b_good(0), "T3c maximum LEN=32 accepted");
        mk(8'h01, 8'd3, 1, 8'h99); send;
        chk(r_good(0) && b_good(0), "T3d minimum LEN=1 accepted");

        // ---- T4: single transient bit-flip in one lane while payload is processed ----
        do_reset;
        mk(8'h20, 8'd40, 16, 8'h0B); inj_idx = 12; inj_lane = 0; inj_mask = BIT_SUM10; send;   // baseline hit
        chk(!b_good(0), "T4 baseline: SEU in lane corrupts delivered result (silent or rejected)");
        mk(8'h20, 8'd41, 16, 8'h0B); inj_idx = 12; inj_lane = 1; inj_mask = BIT_SUM10; send;   // resilient: lane 1 hit
        chk(r_good(0) && r_lflt && !r_lunc, "T4b resilient: SEU in lane 1 outvoted, correct output, fault flagged");
        mk(8'h20, 8'd43, 16, 8'h0C); inj_idx = 12; inj_lane = 0; inj_mask = BIT_SUM10; send;   // lane 0 hit
        chk(r_good(0) && r_lflt && !r_lunc, "T4d resilient: SEU in lane 0 outvoted, correct output, fault flagged");
        mk(8'h20, 8'd44, 16, 8'h0D); inj_idx = 12; inj_lane = 2; inj_mask = BIT_SUM10; send;   // lane 2 hit
        chk(r_good(0) && r_lflt && !r_lunc, "T4e resilient: SEU in lane 2 outvoted, correct output, fault flagged");
        mk(8'h20, 8'd42, 16, 8'h0B); send;
        chk(r_good(0) && !r_lflt, "T4c next frame clean again (lane self-resynchronised at frame boundary)");

        // ---- T4f: coincident faults in two lanes with different effects: no agreeing pair -> rejected, never silent ----
        mk(8'h20, 8'd45, 16, 8'h0E); inj_idx = 12; inj_lane = 0; inj_mask = BIT_SUM10; send;
        mk(8'h20, 8'd46, 16, 8'h0F);                                   // lanes 0 and 1 hit with different bits in the same frame
        inj_idx = 12; inj_lane = 0; inj_mask = BIT_SUM10; fork_flip1 = 1; send;
        chk(r_lunc && !r_lok, "T4f double fault, three different results: frame rejected (detected)");

        // ---- T5: persistent stuck-at in lane 2 -> detected, outvoted, isolated, DMR continues ----
        do_reset; set_stuck(2, BIT_OK, 96'd0);              // lane 2 'ok' bit stuck at 0
        bad = 0; dc0 = r_dc;
        for (k = 0; k < 12; k = k + 1) begin mk(8'h10, k, 8, 8'h60 + k); send; if (!r_good(0)) bad = bad + 1; end
        chk(bad == 0, "T5 all 12 outputs correct while lane 2 is stuck (no data loss)");
        chk(r_iso == 3'b100, "T5 lane 2 isolated -> DMR (iso mask 100)");
        chk(r_dc - dc0 == 6, "T5 stuck lane flagged exactly until isolation threshold (6 detections)");
        mk(8'h10, 8'd50, 8, 8'h11); send;
        chk(r_good(0) && !r_lflt && !r_ldeg, "T5b after isolation: healthy pair agrees, no fault flag, not degraded");

        // ---- T6: DMR mode + transient in a healthy lane -> detected, frame dropped, never silent ----
        mk(8'h10, 8'd51, 8, 8'h12); inj_idx = 8; inj_lane = 0; inj_mask = BIT_SUM10; send;
        chk(r_lunc && !r_lok, "T6 DMR: disagreement detected, frame rejected (fail-safe, no SDC)");
        mk(8'h10, 8'd52, 8, 8'h13); send;
        chk(r_good(0), "T6b DMR continues correctly after the dropped frame");

        // ---- T7: second persistent fault -> SAFE mode (single lane, flagged degraded) ----
        set_stuck(1, BIT_SEQ0, BIT_SEQ0);                       // lane 1 'seq' bit0 stuck at 1 (visible on even seq)
        for (k = 0; k < 40; k = k + 1) begin mk(8'h10, 8'd100 + k, 8, 8'h70 + k); send; end
        chk(r_iso == 3'b110, "T7 lane 1 also isolated -> SAFE mode (iso mask 110)");
        mk(8'h10, 8'd200, 8, 8'h14); send;
        chk(r_good(0) && r_ldeg, "T7b SAFE mode delivers correct data from the surviving lane, flagged degraded");

        // ---- T8: transient burst -> lane isolated -> shadow probation -> re-integration ----
        do_reset;
        bad = 0;
        for (k = 0; k < 6; k = k + 1) begin
            mk(8'h20, 8'd10 + k, 8, 8'h80 + k); inj_idx = 12; inj_lane = 0; inj_mask = BIT_SUM10; send;
            if (!r_good(0)) bad = bad + 1;
        end
        chk(bad == 0, "T8 outputs stay correct during the 6-frame burst in lane 0");
        chk(r_iso == 3'b001, "T8 burst of 6 consecutive upsets isolates lane 0");
        bad = 0;
        for (k = 0; k < 10; k = k + 1) begin mk(8'h20, 8'd30 + k, 8, 8'h90 + k); send; if (!r_good(0)) bad = bad + 1; end
        chk(bad == 0 && r_iso == 3'b000, "T8b faults cease: lane re-integrated after probation, all outputs correct");

        // ---- T9: stream stall mid-frame -> watchdog flushes lanes, next frame survives ----
        do_reset; wd_seen = 0;
        mk(8'h10, 8'd60, 20, 8'h44); partial = 10; send;     // upstream dies after 10 bytes
        repeat (40) @(posedge clk);
        chk(wd_seen == 1, "T9 watchdog fires once on mid-frame stall");
        mk(8'h10, 8'd61, 20, 8'h45); send;
        chk(r_good(0), "T9b resilient: frame after the stall is processed correctly");
        chk(!b_lok, "T9c baseline (no watchdog): frame after the stall is lost");

        // ---- T10: persistent fault + coincident transient on the SAME bit ----
        do_reset; set_stuck(0, BIT_TYPE0, BIT_TYPE0);            // lane 0 type bit0 stuck-at-1
        for (k = 0; k < 10; k = k + 1) begin mk(8'h02, 8'd70 + k, 6, 8'h20 + k); send; end
        mk(8'h02, 8'd90, 6, 8'h55); inj_idx = 4; inj_lane = 1; inj_mask = BIT_TYPE0; send;
        chk(n_lok && n_ltype == 8'h03, "T10 ablation (no isolation): same-bit double fault outvotes good lane -> silent corruption");
        chk(r_lunc && !r_lok, "T10b with isolation: same event is detected and frame rejected (no silent corruption)");

        $display("\n%0d checks, %0d failed", checks, errors);
        if (errors == 0) $display("ALL DIRECTED TESTS PASSED"); else $display("DIRECTED TESTS FAILED");
        $finish;
    end

    initial begin #50000000; $display("TIMEOUT"); $finish; end
endmodule
