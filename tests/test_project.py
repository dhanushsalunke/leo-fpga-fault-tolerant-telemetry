"""pytest suite: golden-model unit tests + RTL regression (needs iverilog; skipped if absent)."""
import os, shutil, subprocess, sys
import pytest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "model")); sys.path.insert(0, os.path.join(ROOT, "scripts"))
import tlm_model as M

IVERILOG = shutil.which("iverilog") or (r"C:\iverilog\bin\iverilog.exe" if os.path.exists(r"C:\iverilog\bin\iverilog.exe") else None)
VVP = shutil.which("vvp") or (r"C:\iverilog\bin\vvp.exe" if os.path.exists(r"C:\iverilog\bin\vvp.exe") else None)
needs_sim = pytest.mark.skipif(IVERILOG is None, reason="iverilog not installed")


def test_crc_check_value():
    assert M.crc16_ccitt(b"123456789") == 0x29B1          # CRC-16/CCITT-FALSE catalogue check value


def test_golden_accepts_valid_and_rejects_corrupt():
    fr = M.build_frame(0x10, 5, bytes(range(1, 9)))
    g = M.golden_parse(fr)
    assert g.ok and g.total == 36 and g.mn == 1 and g.mx == 8 and g.seq == 5
    for i in range(len(fr)):
        for b in range(8):
            bad = bytearray(fr); bad[i] ^= 1 << b
            assert not M.golden_parse(bytes(bad)).ok        # every single-bit error is detected


def test_golden_length_rules():
    assert not M.golden_parse(M.build_frame(1, 0, b"")).ok
    assert not M.golden_parse(M.build_frame(1, 0, bytes(33))).ok
    assert M.golden_parse(M.build_frame(1, 0, bytes(32))).ok


def test_stream_has_one_last_per_frame():
    cyc, fr, gd = M.gen_stream(3, 50)
    assert len(M.sample_cycles(cyc)) == len(fr) == len(gd) == 50


def test_state_vector_layout_matches_rtl():
    src = open(os.path.join(ROOT, "rtl", "tlm_lane.v")).read()
    assert f"STW = {M.STW};" in src
    for name, lsb, w in M.FIELDS:
        key = {"state": "P_ST", "crc": "P_CRC", "crc_hi": "P_CRCHI", "type": "P_TYPE", "seq": "P_SEQ", "len": "P_LEN",
               "cnt": "P_CNT", "sum": "P_SUM", "min": "P_MN", "max": "P_MX", "ok": "P_OK"}[name]
        assert f"{key}={lsb}" in src.replace(" ", "")


@needs_sim
def test_directed_rtl_testbench_passes(tmp_path):
    out = tmp_path / "d.vvp"
    subprocess.run([IVERILOG, "-g2012", "-o", str(out)] + [os.path.join(ROOT, "rtl", f) for f in os.listdir(os.path.join(ROOT, "rtl"))]
                   + [os.path.join(ROOT, "tb", "tb_directed.v")], check=True)
    txt = subprocess.run([VVP, str(out)], capture_output=True, text=True).stdout
    assert "FAIL" not in txt and "ALL DIRECTED TESTS PASSED" in txt


@needs_sim
@pytest.mark.parametrize("design,nl", [("baseline", 1), ("resilient", 3)])
def test_clean_stream_matches_golden_exactly(design, nl, tmp_path):
    import campaign as C
    vvp = C.build(design)
    cyc, fr, gd = M.gen_stream(11, 300, link_err=0.1)
    smp = M.sample_cycles(cyc)
    L, O, _ = C.run_sim(vvp, cyc, [], str(tmp_path))
    r = C.score(design, gd, smp, [], L, O, nl)
    assert r["correct"] == r["frames"] == 300 and r["sdc"] == 0 and r["activated"] == 0


@needs_sim
def test_single_upset_never_corrupts_resilient_output(tmp_path):
    """Sparse single upsets (one flip in one lane every 8th frame, lanes rotating): TMR must never deliver wrong
    data, never drop a good frame, and must flag every activated upset.  (Dense upsets - one per frame - make the
    isolation policy retire lanes; that regime is characterised in the campaign, not asserted here.)"""
    import campaign as C, random
    vvp = C.build("resilient")
    cyc, fr, gd = M.gen_stream(5, 150, link_err=0.0, gap_prob=0.0, idle_prob=0.0)
    smp = M.sample_cycles(cyc)
    rnd = random.Random(1)
    ev = []
    for i, s in enumerate(smp):
        if i % 8: continue
        start = smp[i - 1] + 1 if i else 0
        ev.append((rnd.randrange(start, s + 1), (i // 8) % 3, 1 << rnd.randrange(M.STW), 0, 0))
    ev.sort(key=lambda e: e[0])
    L, O, _ = C.run_sim(vvp, cyc, ev, str(tmp_path))
    r = C.score("resilient", gd, smp, ev, L, O, 3)
    assert r["sdc"] == 0 and r["false_drop"] == 0 and r["correct"] == 150
    assert r["activated"] > 0 and r["detected"] == r["activated"]
