# Fault model and limitations

## What is injected
* **Target:** every flip-flop of a lane (one packed 89-bit state vector: FSM state, running CRC, received CRC byte, type, seq, length, counter, sum, min, max, ok flag).
  The vector layout is shared between `rtl/tlm_lane.v` and `model/tlm_model.py` and checked by a unit test.
* **Transient upset (SEU/MBU model):** XOR of a mask into the next-state value for one cycle; 90 % single-bit, 10 % two adjacent bits. Per-lane, per-cycle probability chosen so the mean is *lambda* upsets per frame.
* **Persistent fault:** a stuck-at-0/1 mask applied to the next-state value of selected bits from a chosen cycle on (also applied during flush/reset, so a flush does not "repair" a stuck bit).
* Injection hooks exist only when the `FI_EN` parameter is 1; with `FI_EN=0` they are not elaborated, and the synthesis numbers are for `FI_EN=0`.

## What is *not* injected or modelled
* Voter, recovery controller, watchdog and output registers (not redundant; a fault there defeats the scheme).
* Configuration-memory upsets, clock/reset glitches, multi-lane common-cause faults (e.g. a shared clock), SET propagation, timing faults.
* Input-side faults other than link bit errors (frames with one flipped bit, detected by the CRC).
* Physical radiation effects, particle flux or cross-sections. Rates are accelerated *per-frame* rates, not orbital upset rates.

## Known limitations of the design
* Two lanes failing identically (same stuck bit on two lanes, identical wrong vectors) outvote the good lane. Isolation reduces but cannot remove this (E3).
* Isolation needs evidence; a stuck bit that shows up in fewer than about 6 of every 16 frames is never isolated (the vote still masks it).
* DMR mode rejects (does not correct) on disagreement: availability is traded for integrity. SAFE mode has no detection at all and says so with the `degraded` flag.
* Attribution uses the shadow lane as a tie-breaker; if that lane is itself faulty, blame can be ambiguous (the campaign shows the resulting extra rejects).
* Resource cost is x8.6 LUTs / x2.9 FFs (Yosys estimate). No timing analysis.
* Frames rely on external `in_last` framing. Without it, a corrupted LEN would need an inter-byte timeout in the parser itself.
