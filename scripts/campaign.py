"""Fault-injection campaigns: baseline (1 lane) vs resilient (TMR + recovery controller).

Runs Icarus Verilog on stimulus/fault plans from model/tlm_model.py, scores the logged
DUT outputs against the golden model, and writes results/*.csv|md|png.

    python scripts/campaign.py            # full campaign (a few minutes)
    python scripts/campaign.py --quick    # reduced size, used by CI
"""
import argparse, csv, os, random, shutil, statistics, subprocess, sys, tempfile
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "model"))
import tlm_model as M

IVERILOG = shutil.which("iverilog") or r"C:\iverilog\bin\iverilog.exe"
VVP = shutil.which("vvp") or r"C:\iverilog\bin\vvp.exe"
BUILD = os.path.join(ROOT, "build")
RTL = [os.path.join(ROOT, "rtl", f) for f in ("tlm_lane.v", "tlm_baseline_top.v", "tlm_resilient_top.v")]
TB = os.path.join(ROOT, "tb", "tb_campaign.v")


def build(design, iso_en=1):
    os.makedirs(BUILD, exist_ok=True)
    out = os.path.join(BUILD, f"{design}_iso{iso_en}.vvp")
    defs = ["-DRESILIENT", f"-DISO_EN={iso_en}"] if design == "resilient" else []
    subprocess.run([IVERILOG, "-g2012", "-o", out, *defs, *RTL, TB], check=True)
    return out


def run_sim(vvp, cycles, events, workdir):
    pfx = os.path.join(workdir, "run")
    nev = M.write_mem_files(pfx, cycles, events)
    log = pfx + ".log"
    subprocess.run([VVP, vvp, f"+pfx={pfx}", f"+ncyc={len(cycles)}", f"+nev={nev}", f"+log={log}"],
                   check=True, stdout=subprocess.DEVNULL)
    L, O, cov = [], [], None
    for line in open(log):
        t = line.split()
        if t[0] == "L": L.append((int(t[1]), [int(x, 16) for x in t[2:5]]))
        elif t[0] == "O": O.append([int(x) for x in t[1:]])
        elif t[0] == "C": cov = [int(x) for x in t[1:4]]
    return L, O, cov


def dec(v):
    return (v >> 48 & 1, v >> 40 & 0xFF, v >> 32 & 0xFF, v >> 16 & 0xFFFF, v >> 8 & 0xFF, v & 0xFF)


def lane_ok(v, g):
    d = dec(v)
    return (d == g.key()) if g.ok else (d[0] == 0)


def score(design, goldens, samples, events, L, O, nlanes):
    """Per-frame scoring.  Returns dict of counters and latency lists."""
    assert len(L) == len(goldens) == len(O), (len(L), len(goldens), len(O))
    r = dict(frames=len(goldens), events=len(events), delivered_ok=0, correct=0, sdc=0, false_drop=0,
             activated=0, detected=0, corrected=0, uncorr=0, degraded=0, wd=0, det_lat=[], resync=[],
             sdc_hidden=0, sdc_flagged=0, iso_first=None, iso_max=0, n_iso_frames=0, iso_seen=set())
    prev = -1
    ev_by_frame = []
    for i, s in enumerate(samples):
        ev_by_frame.append([e for e in events if prev < e[0] <= s]); prev = s
    for i, g in enumerate(goldens):
        dev = [not lane_ok(L[i][1][k], g) for k in range(nlanes)]
        o = O[i]
        ok, otype, oseq, osum, omin, omax = o[1:7]
        out_dec = (ok, otype, oseq, osum, omin, omax)
        if g.ok: out_good = (out_dec == g.key())
        else:    out_good = (ok == 0)
        if ok == 1: r["delivered_ok"] += 1
        if out_good: r["correct"] += 1
        elif ok == 1:
            r["sdc"] += 1
            if design == "resilient" and o[9]: r["sdc_flagged"] += 1   # delivered with `degraded` flag set
            else: r["sdc_hidden"] += 1
        else: r["false_drop"] += 1
        flag = bool(o[7] or o[8]) if design == "resilient" else False
        if design == "resilient":
            r["uncorr"] += o[8]; r["degraded"] += o[9]; r["wd"] += o[11]
            r["iso_seen"].add(o[10])
            if o[10] and r["iso_first"] is None: r["iso_first"] = i
            if o[10]: r["n_iso_frames"] += 1
        if any(dev):
            r["activated"] += 1
            if flag: r["detected"] += 1
            if out_good and flag: r["corrected"] += 1
            evs = [e for e in ev_by_frame[i] if dev[e[1]] and e[2] != 0]
            if flag and evs:
                r["det_lat"].append(samples[i] + 2 - min(e[0] for e in evs))
            if evs:   # resync: first later frame in which every lane is correct again
                for j in range(i + 1, len(goldens)):
                    if not any(not lane_ok(L[j][1][k], goldens[j]) for k in range(nlanes)):
                        r["resync"].append(samples[j] + 2 - min(e[0] for e in evs)); break
    return r


def stuck_event(cycle, lane, bit, val):
    return (cycle, lane, 0, 1 << bit, (val << bit))


def pct(a, b):
    return 100.0 * a / b if b else float("nan")


def med(x):
    return statistics.median(x) if x else float("nan")


def run_exp(quick):
    nfr = 600 if quick else 2000
    seeds = [1] if quick else [1, 2]
    work = tempfile.mkdtemp(prefix="tlmcamp_")
    vb, vr, vr0 = build("baseline"), build("resilient", 1), build("resilient", 0)
    rows = []
    # ---------------- E1: transient SEU sweep ----------------
    for lam in ([0.1, 0.5] if quick else [0.01, 0.05, 0.1, 0.5, 1.0]):
        agg = {d: None for d in ("baseline", "resilient")}
        for design, vvp, nl in (("baseline", vb, 1), ("resilient", vr, 3)):
            tot = dict(frames=0, events=0, delivered_ok=0, sdc=0, sdc_hidden=0, sdc_flagged=0, false_drop=0, activated=0, detected=0,
                       corrected=0, uncorr=0, det_lat=[], resync=[], correct=0)
            for sd in seeds:
                cyc, fr, gd = M.gen_stream(sd, nfr)
                smp = M.sample_cycles(cyc)
                ev = M.gen_flip_plan(sd, cyc, nl, lam)
                L, O, _ = run_sim(vvp, cyc, ev, work)
                r = score(design, gd, smp, ev, L, O, nl)
                for k in tot:
                    tot[k] = tot[k] + r[k] if not isinstance(tot[k], list) else tot[k] + r[k]
            agg[design] = tot
            rows.append(dict(exp="E1-SEU", lam=lam, design=design, frames=tot["frames"], injected=tot["events"],
                             activated=tot["activated"], detected=tot["detected"],
                             coverage_pct=pct(tot["detected"], tot["activated"]) if design == "resilient" else 0.0,
                             sdc=tot["sdc"], sdc_hidden=tot["sdc_hidden"], sdc_flagged=tot["sdc_flagged"], sdc_pct=pct(tot["sdc_hidden"], tot["delivered_ok"]),
                             false_drop=tot["false_drop"], uncorr=tot["uncorr"],
                             det_lat_med=med(tot["det_lat"]), det_lat_max=max(tot["det_lat"], default=float("nan")),
                             resync_med=med(tot["resync"]), resync_max=max(tot["resync"], default=float("nan"))))
    # ---------------- E2: permanent stuck-at (random bit/lane/value) ----------------
    n_trials = 8 if quick else 30
    modes = {"baseline": (vb, 1), "resilient_no_iso": (vr0, 3), "resilient": (vr, 3)}
    acc = {k: dict(trials=0, sdc=0, sdc_hidden=0, sdc_flagged=0, delivered=0, false_drop=0, uncorr=0, degraded=0, iso_frames=[], isolated=0, frames=0)
           for k in modes}
    cov_modes = set()
    for t in range(n_trials):
        rnd = random.Random(1000 + t)
        cyc, fr, gd = M.gen_stream(500 + t, 1200 if not quick else 400, link_err=0.05)
        smp = M.sample_cycles(cyc)
        f1 = rnd.randrange(60, 100)
        b1, v1, lane1 = rnd.randrange(M.STW), rnd.randrange(2), rnd.randrange(3)
        b2, v2 = rnd.randrange(M.STW), rnd.randrange(2)
        lane2 = (lane1 + 1 + rnd.randrange(2)) % 3
        f2 = len(smp) // 2
        for name, (vvp, nl) in modes.items():
            ev = [stuck_event(smp[f1] - 20, lane1 if nl == 3 else 0, b1, v1)]
            if nl == 3: ev.append(stuck_event(smp[f2] - 20, lane2, b2, v2))   # second persistent fault
            ev += M.gen_flip_plan(500 + t, cyc, nl, 0.05)
            ev.sort(key=lambda e: e[0])
            L, O, cov = run_sim(vvp, cyc, ev, work)
            r = score("resilient" if nl == 3 else "baseline", gd, smp, ev, L, O, nl)
            a = acc[name]
            a["trials"] += 1; a["sdc"] += r["sdc"]; a["sdc_hidden"] += r["sdc_hidden"]; a["sdc_flagged"] += r["sdc_flagged"]; a["delivered"] += r["delivered_ok"]; a["false_drop"] += r["false_drop"]
            a["uncorr"] += r["uncorr"]; a["degraded"] += r["degraded"]; a["frames"] += r["frames"]
            if r["iso_first"] is not None: a["isolated"] += 1; a["iso_frames"].append(r["iso_first"] - f1)
            cov_modes |= r["iso_seen"]
    for name, a in acc.items():
        rows.append(dict(exp="E2-stuck", design=name, frames=a["frames"], trials=a["trials"],
                         sdc=a["sdc"], sdc_hidden=a["sdc_hidden"], sdc_flagged=a["sdc_flagged"], sdc_pct=pct(a["sdc_hidden"], a["delivered"]), false_drop=a["false_drop"],
                         uncorr=a["uncorr"], degraded=a["degraded"], isolated_trials=a["isolated"],
                         iso_frames_med=med(a["iso_frames"]), iso_frames_max=max(a["iso_frames"], default=float("nan"))))
    # ---------------- E3: persistent fault + coincident transient on the SAME bit ----------------
    n3 = 6 if quick else 15
    for name, vvp in (("resilient_no_iso", vr0), ("resilient", vr)):
        sdc = deliv = unc = hid = flg = 0
        for t in range(n3):
            cyc, fr, gd = M.gen_stream(900 + t, 600 if quick else 1500, link_err=0.0)
            smp = M.sample_cycles(cyc)
            ev = [stuck_event(smp[10] - 20, 0, 28, 1)]                    # lane0 'type' bit0 stuck-at-1
            rr = random.Random(t)
            for c in range(len(cyc)):                                       # lane1 SEUs on same bit (~0.15 / frame)
                if rr.random() < 0.15 * len(smp) / len(cyc): ev.append((c, 1, 1 << 28, 0, 0))
            ev.sort(key=lambda e: e[0])
            L, O, _ = run_sim(vvp, cyc, ev, work)
            r = score("resilient", gd, smp, ev, L, O, 3)
            sdc += r["sdc"]; hid += r["sdc_hidden"]; flg += r["sdc_flagged"]; deliv += r["delivered_ok"]; unc += r["uncorr"]
        rows.append(dict(exp="E3-samebit", design=name, frames=n3 * (600 if quick else 1500), trials=n3, sdc=sdc,
                         sdc_hidden=hid, sdc_flagged=flg, sdc_pct=pct(hid, deliv), uncorr=unc))
    # ---------------- FSM / mode coverage (from the E2 runs + one clean run) ----------------
    cyc, fr, gd = M.gen_stream(7, 200)
    _, _, cov = run_sim(vr, cyc, [], work)
    cov_states = [bin(c).count("1") for c in cov]
    shutil.rmtree(work, ignore_errors=True)
    return rows, dict(iso_masks_seen=sorted(cov_modes), fsm_states_visited_per_lane=cov_states)


def write_outputs(rows, extra):
    os.makedirs(os.path.join(ROOT, "results"), exist_ok=True)
    cols = sorted({k for r in rows for k in r}, key=lambda k: (k not in ("exp", "design", "lam"), k))
    with open(os.path.join(ROOT, "results", "campaign_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(rows)
    fmt = lambda v: f"{v:.2f}" if isinstance(v, float) else str(v)
    md = ["# Fault-injection campaign results (RTL simulation, Icarus Verilog)\n",
          "Auto-generated by `scripts/campaign.py`. Simulation of RTL with injected flip-flop faults; **not** radiation testing.\n",
          "\n## E1 - transient single/multi-bit upsets (lambda = mean injected events per lane per frame)\n",
          "| lambda | design | frames | injected | activated | detected | coverage % | unflagged SDC | flagged-degraded SDC | unflagged SDC % of delivered | false drops | uncorrectable | det. latency med/max (cyc) | resync med/max (cyc) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in [r for r in rows if r["exp"] == "E1-SEU"]:
        md.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {}/{} | {}/{} |".format(
            r["lam"], r["design"], r["frames"], r["injected"], r["activated"], r["detected"],
            fmt(r["coverage_pct"]) if r["design"] == "resilient" else "n/a", r["sdc_hidden"], r["sdc_flagged"], fmt(r["sdc_pct"]),
            r["false_drop"], r["uncorr"], fmt(r["det_lat_med"]), fmt(r["det_lat_max"]),
            fmt(r["resync_med"]), fmt(r["resync_max"])))
    md += ["\n## E2 - persistent stuck-at faults (random bit/lane/value; 2nd persistent fault in mid-run; background SEU lambda=0.05)\n",
           "| design | trials | frames | unflagged SDC | flagged-degraded SDC | unflagged SDC % of delivered | false drops | uncorrectable | degraded frames | trials with isolation | frames to isolation med/max |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in [r for r in rows if r["exp"] == "E2-stuck"]:
        md.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {}/{} |".format(
            r["design"], r["trials"], r["frames"], r["sdc_hidden"], r["sdc_flagged"], fmt(r["sdc_pct"]), r["false_drop"], r["uncorr"], r["degraded"],
            r["isolated_trials"], fmt(r["iso_frames_med"]), fmt(r["iso_frames_max"])))
    md += ["\n## E3 - persistent stuck-at in lane 0 + coincident SEUs on the same bit in lane 1\n",
           "| design | trials | frames | unflagged SDC | flagged-degraded SDC | unflagged SDC % of delivered | uncorrectable (detected, dropped) |", "|---|---|---|---|---|---|---|"]
    for r in [r for r in rows if r["exp"] == "E3-samebit"]:
        md.append(f"| {r['design']} | {r['trials']} | {r['frames']} | {r['sdc_hidden']} | {r['sdc_flagged']} | {fmt(r['sdc_pct'])} | {r['uncorr']} |")
    md += ["\n## Coverage bookkeeping\n", f"- isolation masks observed in E2 runs: {extra['iso_masks_seen']} (0 = TMR; one bit set = DMR; two bits set = SAFE)",
           f"- FSM states visited per lane in a clean run: {extra['fsm_states_visited_per_lane']} of 9\n"]
    open(os.path.join(ROOT, "results", "campaign_results.md"), "w").write("\n".join(md) + "\n")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        e1 = [r for r in rows if r["exp"] == "E1-SEU"]
        fig, ax = plt.subplots(figsize=(5.2, 3.3))
        for d, c in (("baseline", "#b03a2e"), ("resilient", "#1f618d")):
            xs = [r["lam"] for r in e1 if r["design"] == d]; ys = [r["sdc_hidden"] / max(r["frames"], 1) * 100 for r in e1 if r["design"] == d]
            ax.plot(xs, ys, "o-", color=c, label=d)
        ax.set_xscale("log"); ax.set_xlabel("injected upsets per lane per frame"); ax.set_ylabel("silent corruption (% of frames)")
        ax.set_title("Silent data corruption vs upset rate (RTL simulation)", fontsize=9); ax.legend(); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(ROOT, "results", "sdc_vs_upset_rate.png"), dpi=160)
    except Exception as e:  # plotting is optional
        print("plot skipped:", e)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--quick", action="store_true"); a = ap.parse_args()
    rows, extra = run_exp(a.quick)
    write_outputs(rows, extra)
    print(open(os.path.join(ROOT, "results", "campaign_results.md")).read())
