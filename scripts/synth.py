"""Yosys synthesis statistics (Xilinx 7-series technology mapping, no place-and-route).

Reports LUT / FF / CARRY counts for the baseline and the resilient top with fault-injection
hooks disabled (FI_EN=0, the default).  This is *logic-resource* data only: no timing,
no routing, no vendor tool (Vivado) run.  Requires `yosys` or `yowasp-yosys` (pip).
"""
import os, re, shutil, subprocess, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
Y = shutil.which("yosys") or shutil.which("yowasp-yosys") or os.path.join(
    os.path.dirname(sys.executable), "Scripts", "yowasp-yosys.exe")
SRC = " ".join(os.path.join("rtl", f) for f in ("tlm_lane.v", "tlm_baseline_top.v", "tlm_resilient_top.v"))


def stats(top):
    os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
    stat = os.path.join("build", f"{top}.stat")
    script = (f"read_verilog {SRC}; hierarchy -check -top {top}; "
              f"synth_xilinx -family xc7 -top {top} -flatten; check -assert; tee -o {stat} stat")
    subprocess.run([Y, "-q", "-p", script], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    txt = open(os.path.join(ROOT, stat)).read()
    cnt = {m.group(2): int(m.group(1)) for m in re.finditer(r"^\s+(\d+)\s+(\S+)\s*$", txt, re.M)}
    lut = sum(v for k, v in cnt.items() if re.fullmatch(r"LUT\d", k))
    ff = sum(v for k, v in cnt.items() if k.startswith("FD"))
    return dict(lut=lut, ff=ff, carry4=cnt.get("CARRY4", 0), muxf=cnt.get("MUXF7", 0) + cnt.get("MUXF8", 0))


if __name__ == "__main__":
    b, r = stats("tlm_baseline_top"), stats("tlm_resilient_top")
    md = ["# Logic-resource comparison (Yosys, `synth_xilinx -family xc7`, fault-injection hooks disabled)\n",
          "Logic mapping only - **no place-and-route, no timing analysis, no Vivado run**. Numbers are Yosys estimates.\n",
          "| design | LUTs | FFs | CARRY4 | MUXF7/8 |", "|---|---|---|---|---|",
          f"| baseline (1 lane) | {b['lut']} | {b['ff']} | {b['carry4']} | {b['muxf']} |",
          f"| resilient (3 lanes + voter + controller + watchdog) | {r['lut']} | {r['ff']} | {r['carry4']} | {r['muxf']} |",
          f"| overhead | x{r['lut'] / b['lut']:.2f} | x{r['ff'] / b['ff']:.2f} | x{r['carry4'] / max(b['carry4'], 1):.2f} | |", ""]
    open(os.path.join(ROOT, "results", "synth_stats.md"), "w").write("\n".join(md))
    print("\n".join(md))
