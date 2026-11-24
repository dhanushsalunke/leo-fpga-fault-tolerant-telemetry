# Logic-resource comparison (Yosys, `synth_xilinx -family xc7`, fault-injection hooks disabled)

Logic mapping only - **no place-and-route, no timing analysis, no Vivado run**. Numbers are Yosys estimates.

| design | LUTs | FFs | CARRY4 | MUXF7/8 |
|---|---|---|---|---|
| baseline (1 lane) | 118 | 140 | 8 | 0 |
| resilient (3 lanes + voter + controller + watchdog) | 1015 | 410 | 42 | 3 |
| overhead | x8.60 | x2.93 | x5.25 | |
