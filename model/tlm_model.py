"""Reference (golden) model, stimulus and fault-plan generators for the telemetry pipeline.

The golden model is a *specification-level* parser (not a transliteration of the RTL FSM):
a frame is valid iff it starts with A5 5A, has LEN in 1..32, has exactly 5+LEN+2 bytes
and carries a correct CRC-16/CCITT-FALSE over TYPE|SEQ|LEN|PAYLOAD.
"""
import random
from dataclasses import dataclass

STW = 89  # width of the lane state vector in RTL (fault-injection address space)
# (name, lsb, width) of each field in the lane state vector; must match rtl/tlm_lane.v
FIELDS = [("state", 0, 4), ("crc", 4, 16), ("crc_hi", 20, 8), ("type", 28, 8),
          ("seq", 36, 8), ("len", 44, 6), ("cnt", 50, 6), ("sum", 56, 16),
          ("min", 72, 8), ("max", 80, 8), ("ok", 88, 1)]


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(ptype: int, seq: int, payload: bytes) -> bytes:
    body = bytes([ptype, seq, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return b"\xA5\x5A" + body + bytes([crc >> 8, crc & 0xFF])


@dataclass
class Golden:
    ok: bool
    ptype: int = 0
    seq: int = 0
    total: int = 0
    mn: int = 0
    mx: int = 0

    def key(self):
        return (1, self.ptype, self.seq, self.total, self.mn, self.mx) if self.ok else (0,)


def golden_parse(frame: bytes) -> Golden:
    if len(frame) < 7 or frame[0] != 0xA5 or frame[1] != 0x5A:
        return Golden(False)
    ln = frame[4]
    if not 1 <= ln <= 32 or len(frame) != 5 + ln + 2:
        return Golden(False)
    body, crc = frame[2:5 + ln], (frame[5 + ln] << 8) | frame[6 + ln]
    if crc16_ccitt(body) != crc:
        return Golden(False)
    pl = frame[5:5 + ln]
    return Golden(True, frame[2], frame[3], sum(pl), min(pl), max(pl))


def gen_stream(seed, n_frames, link_err=0.05, gap_prob=0.1, max_gap=4, idle_prob=0.05):
    """Return (cycles, frames, goldens).  cycles[i] = (valid, last, data).  One frame -> one in_last."""
    rnd = random.Random(seed)
    cycles, frames, goldens = [], [], []
    seq = rnd.randrange(256)
    for _ in range(n_frames):
        ln = rnd.choice([1, 2, 4, 8, 16, 24, 32]) if rnd.random() < 0.5 else rnd.randint(1, 32)
        fr = bytearray(build_frame(rnd.choice([0x01, 0x02, 0x10, 0x20, 0x7F]), seq, bytes(rnd.randrange(256) for _ in range(ln))))
        seq = (seq + 1) & 0xFF
        if rnd.random() < link_err:                      # link-layer corruption (CRC must catch it)
            i = rnd.randrange(len(fr)); fr[i] ^= 1 << rnd.randrange(8)
        fr = bytes(fr)
        frames.append(fr); goldens.append(golden_parse(fr))
        for j, b in enumerate(fr):
            while rnd.random() < idle_prob:              # occasional valid=0 bubble inside a frame
                cycles.append((0, 0, 0))
            cycles.append((1, 1 if j == len(fr) - 1 else 0, b))
        if rnd.random() < gap_prob:
            cycles.extend([(0, 0, 0)] * rnd.randint(1, max_gap))
    cycles.extend([(0, 0, 0)] * 8)
    return cycles, frames, goldens


def sample_cycles(cycles):
    """Cycle index of each in_last (the DUT result is valid in cycle idx+1, out_valid in idx+2)."""
    return [i for i, (v, l, _) in enumerate(cycles) if v and l]


def gen_flip_plan(seed, cycles, lanes, rate_per_frame, mbu_prob=0.1, fields=None):
    """Poisson-like SEU plan: per lane, per cycle probability so the mean is `rate_per_frame` per frame.
    Returns events (cycle, lane, flip_mask, 0, 0), sorted by cycle."""
    rnd = random.Random(seed ^ 0x5EED)
    n_frames = len(sample_cycles(cycles))
    p = rate_per_frame * n_frames / len(cycles)
    ev = []
    for lane in range(lanes):
        for c in range(len(cycles)):
            if rnd.random() < p:
                if fields:
                    name = rnd.choice(fields)
                    f = next(x for x in FIELDS if x[0] == name)
                    bit = f[1] + rnd.randrange(f[2])
                else:
                    bit = rnd.randrange(STW)
                m = 1 << bit
                if rnd.random() < mbu_prob and bit + 1 < STW:
                    m |= 1 << (bit + 1)
                ev.append((c, lane, m, 0, 0))
    ev.sort(key=lambda e: e[0])
    return ev


def write_mem_files(prefix, cycles, events):
    with open(prefix + "_stim.mem", "w") as f:
        for v, l, d in cycles:
            f.write(f"{(v << 9) | (l << 8) | d:03x}\n")
    with open(prefix + "_ev.mem", "w") as fc, open(prefix + "_evl.mem", "w") as fl, \
         open(prefix + "_evf.mem", "w") as ff, open(prefix + "_evm.mem", "w") as fm, \
         open(prefix + "_evv.mem", "w") as fv:
        for c, lane, fl_m, sm, sv in (events or [(0, 0, 0, 0, 0)]):
            fc.write(f"{c:08x}\n"); fl.write(f"{lane:x}\n")
            ff.write(f"{fl_m:024x}\n"); fm.write(f"{sm:024x}\n"); fv.write(f"{sv:024x}\n")
    return max(len(events), 0)
