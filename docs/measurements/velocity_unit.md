# Present_Velocity unit (2026-11-08)

## Summary

Present_Velocity (register 58, sign bit 15) is in **counts per second**, in the same
direction as Present_Position: a least-squares fit against the finite difference of
position gives v_reg = 0.996-0.997 x v_fd, R^2 0.996-0.997, over 3 runs (2839 moving
samples, 0 read failures). read() converts it with the same scale as position:
rad/s = sign x counts/s / (4096 / 2 pi).

## Setup

- Test: so101_hardware/test/test_sts_bus.cpp, StsBusHardware.VelocityUnitAgainstFiniteDifference
  (read-only; skips unless Torque_Enable on ID 1 reads 0).
- Joint: shoulder_pan (ID 1), moved by hand, slow and fast, both directions, 10 s per run.
  Its range (761..3410 counts) never crosses the encoder wrap.
- Sampling: SYNC READ of registers 56..61 on a 100 Hz grid (sleep_until), each sample
  timestamped with steady_clock after the reply.
- v_fd: central difference of Present_Position, (p[k+1] - p[k-1]) / (t[k+1] - t[k-1]),
  in counts/s. Samples with |v_fd| < 50 counts/s (~4.4 deg/s) are excluded as noise.
- Fit: least squares through the origin, a = sum(v_reg * v_fd) / sum(v_fd^2).
- Port: QinHeng USB serial adapter (cdc_acm), 1 000 000 baud, nothing else on the bus.

## Results

| Run | Moving samples | a | R^2 | Sign agreement | Read failures |
|---|---|---|---|---|---|
| 1 | 866 | 0.997 | 0.996 | 847/866 | 0 |
| 2 | 986 | 0.997 | 0.997 | 977/986 | 0 |
| 3 | 987 | 0.996 | 0.996 | 979/987 | 0 |

## Interpretation

- a = 1 within 0.4% and R^2 > 0.99: one register unit is one count per second.
- Sign agreement 97.8-99.2%: sign bit 15 decodes correctly and the register's direction
  matches Present_Position. A wrong sign bit or reversed direction would give a ~ -1.
- The causes of the ~0.3% scale shortfall and of the 8-19 disagreeing samples per run
  were not investigated.

## Limits of this measurement

- Torque off, joint moved by hand: velocity under servo control (torque on) is not covered
  and is checked again at first powered motion (M2).
- a = 1 within 0.4% and R^2 > 0.99: one register unit is one count per second.
- Sign agreement 97.8-99.2%: sign bit 15 decodes correctly and the register's direction
  matches Present_Position. A wrong sign bit or reversed direction would give a ~ -1.
- The causes of the ~0.3% scale shortfall and of the 8-19 disagreeing samples per run
  were not investigated.

## Limits of this measurement

- Torque off, joint moved by hand: velocity under servo control (torque on) is not covered
  and is checked again at first powered motion (M2).
- One joint (shoulder_pan). The register is the same on all six servos; per-joint signs
  are verified separately against RViz (M2).
