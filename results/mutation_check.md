# Mutation sanity check of the directed testbench

| mutant | failing checks | killed |
|---|---|---|
| TMR vote replaced by lane 0 | 2 | yes |
| fault flag tied low (no detection) | 4 | yes |
| no-agreeing-pair case not rejected | 1 | yes |
| lane isolation disabled | 7 | yes |
| watchdog never arms | 2 | yes |
| CRC compare ignores low byte | 1 | yes |
| LEN upper bound removed | 1 | yes |
| DMR mismatch not rejected | 3 | yes |

survivors: 0
