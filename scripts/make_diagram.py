"""Draws docs/architecture.png (block diagram of the resilient telemetry processor)."""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
fig, ax = plt.subplots(figsize=(9.2, 4.6)); ax.set_xlim(0, 100); ax.set_ylim(0, 50); ax.axis("off")


def box(x, y, w, h, text, fc="#eaf2f8", ec="#1b4f72", fs=8, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.25,rounding_size=0.8", fc=fc, ec=ec, lw=1.1))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, weight="bold" if bold else "normal")


def arrow(x1, y1, x2, y2, txt=None, dashed=False):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", lw=1.0, color="#333", ls="--" if dashed else "-"))
    if txt: ax.text((x1 + x2) / 2, (y1 + y2) / 2 + 1.2, txt, fontsize=6.5, ha="center", color="#333")


box(1, 20, 12, 10, "byte stream\nin_valid / in_data\nin_last", fc="#fdf2e9", ec="#a04000")
for i, y in enumerate((36, 21, 6)):
    box(20, y, 30, 9, f"Lane {i}\nparser FSM | CRC-16 | sum/min/max\n(89-bit state vector)", fc="#d6eaf8", fs=7)
    arrow(13, 25, 20, y + 4.5)
    arrow(50, y + 4.5, 58, 25 + (y - 21) * 0.18, None)
box(58, 15, 16, 20, "Voter\nword-level 2-of-3\n+ per-lane\nmismatch", fc="#d5f5e3", ec="#1e8449", bold=True)
box(77, 28, 22, 14, "Recovery controller\n16-frame mismatch history\nisolate / probation /\nre-integrate", fc="#fcf3cf", ec="#9a7d0a", fs=7)
box(77, 8, 22, 14, "Mode / output\nTMR -> DMR -> SAFE\nout_ok, st_fault,\nst_uncorr, st_degraded", fc="#f5eef8", ec="#6c3483", fs=7)
box(20, 46.2, 30, 3.3, "Stream watchdog (idle-in-frame timeout)", fc="#fadbd8", ec="#922b21", fs=7)
arrow(74, 30, 77, 34); arrow(77, 31, 74, 28, None)
arrow(74, 22, 77, 15)
arrow(88, 28, 88, 22, "mode")
arrow(77, 39, 50, 40.5, "isolate / flush lane", dashed=True)
arrow(35, 46.2, 35, 45.2, None, dashed=True)
ax.text(1, 2, "Verification-only fault-injection hooks (FI_EN=1) flip or stick any lane flip-flop; disabled in synthesis.\n"
        "Voter, controller and watchdog are not redundant (single points of failure). Lane isolation is a controller-level\n"
        "abstraction of reconfiguration/scrubbing - no dynamic partial reconfiguration is implemented.", fontsize=6.5, color="#555")
fig.savefig(os.path.join(ROOT, "docs", "architecture.png"), dpi=170, bbox_inches="tight")
