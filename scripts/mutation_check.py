"""Mutation sanity check: deliberately break the resilient RTL and confirm the directed tests notice.

A directed test-suite that passes on broken hardware is worthless; this script demonstrates that
each mutant is *killed* (>=1 FAIL).  Mutants are applied to a temporary copy; rtl/ is untouched.
"""
import os, shutil, subprocess, sys, tempfile
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IVERILOG = shutil.which("iverilog") or r"C:\iverilog\bin\iverilog.exe"
VVP = shutil.which("vvp") or r"C:\iverilog\bin\vvp.exe"
MUTANTS = [
    ("TMR vote replaced by lane 0", "tlm_resilient_top.v", "2'd3: begin sel = maj; flt = ~all_eq; uncorr = ~any_pair; end", "2'd3: begin sel = v0; flt = ~all_eq; uncorr = ~any_pair; end"),
    ("fault flag tied low (no detection)", "tlm_resilient_top.v", "flt = ~all_eq; uncorr", "flt = 1'b0; uncorr"),
    ("no-agreeing-pair case not rejected", "tlm_resilient_top.v", "uncorr = ~any_pair; end", "uncorr = 1'b0; end"),
    ("lane isolation disabled", "tlm_resilient_top.v", "wire can_iso = (ISO_EN != 0) && (nh >= 2'd2);", "wire can_iso = 1'b0;"),
    ("watchdog never arms", "tlm_resilient_top.v", "in_pkt <= !in_last;", "in_pkt <= 1'b0;"),
    ("CRC compare ignores low byte", "tlm_lane.v", "{st[P_CRCHI +: 8], in_data} == f_crc", "{st[P_CRCHI +: 8], f_crc[7:0]} == f_crc"),
    ("LEN upper bound removed", "tlm_lane.v", "in_data > 8'd32", "in_data > 8'd63"),
    ("DMR mismatch not rejected", "tlm_resilient_top.v", "if (uncorr) out_ok <= 1'b0;", ""),
]


def run(src_dir):
    out = os.path.join(src_dir, "m.vvp")
    srcs = [os.path.join(src_dir, f) for f in ("tlm_lane.v", "tlm_baseline_top.v", "tlm_resilient_top.v")]
    subprocess.run([IVERILOG, "-g2012", "-o", out, *srcs, os.path.join(ROOT, "tb", "tb_directed.v")], check=True)
    txt = subprocess.run([VVP, out], capture_output=True, text=True).stdout
    return txt.count("FAIL:"), "ALL DIRECTED TESTS PASSED" in txt


if __name__ == "__main__":
    d = tempfile.mkdtemp()
    for f in os.listdir(os.path.join(ROOT, "rtl")): shutil.copy(os.path.join(ROOT, "rtl", f), d)
    fails, ok = run(d)
    print(f"unmodified RTL: {fails} failures, passed={ok}")
    assert ok, "directed tests must pass on the unmodified RTL"
    survivors = 0
    rows = ["| mutant | failing checks | killed |", "|---|---|---|"]
    for name, f, old, new in MUTANTS:
        for g in os.listdir(os.path.join(ROOT, "rtl")): shutil.copy(os.path.join(ROOT, "rtl", g), d)
        p = os.path.join(d, f); t = open(p).read()
        assert old in t, f"mutation site not found: {name}"
        open(p, "w").write(t.replace(old, new))
        fails, ok = run(d)
        killed = fails > 0 or not ok
        survivors += not killed
        rows.append(f"| {name} | {fails} | {'yes' if killed else '**NO**'} |")
        print(rows[-1])
    open(os.path.join(ROOT, "results", "mutation_check.md"), "w").write(
        "# Mutation sanity check of the directed testbench\n\n" + "\n".join(rows) + f"\n\nsurvivors: {survivors}\n")
    shutil.rmtree(d, ignore_errors=True)
    sys.exit(1 if survivors else 0)
