# DisplayPort Link Training Model

[![CI](https://github.com/ABHISHEKSOUNDALGEKAR/DP-link-training-model/actions/workflows/ci.yml/badge.svg)](https://github.com/ABHISHEKSOUNDALGEKAR/DP-link-training-model/actions/workflows/ci.yml)

A transaction-level C++17 model of **DisplayPort 8b/10b link training**, built as a
study project to learn the protocol and to practice the structure of a
verification IP: a bus functional model, a passive monitor, protocol checkers,
functional coverage, error injection, a reference-model scoreboard, and directed
plus constrained-random tests.

**Status:** 116/116 tests pass (16 directed + 100 random seeds), 100% functional
coverage, clean under AddressSanitizer and UBSan, verified on 5,000 additional
random seeds.

> This is a learning model, not a spec-complete implementation. See
> [Simplifications](#simplifications).

---

## Contents

- [Quick start](#quick-start)
- [Architecture](#architecture)
- [Link-training flow](#link-training-flow)
- [Protocol checks](#protocol-checks)
- [Functional coverage](#functional-coverage)
- [Tests](#tests)
- [Reading the log](#reading-the-log)
- [Simplifications](#simplifications)
- [Roadmap](#roadmap)

---

## Quick start

### macOS

1. Open Terminal.
2. Install the command-line compiler tools (skip if already installed):
   ```bash
   xcode-select --install
   ```
3. Confirm a compiler is available (on macOS, `g++` is Apple clang, which works):
   ```bash
   g++ --version
   ```
4. Clone the repository and enter it:
   ```bash
   git clone https://github.com/ABHISHEKSOUNDALGEKAR/DP-link-training-model.git
   cd DP-link-training-model
   ```
5. Build:
   ```bash
   make
   ```
6. Run everything:
   ```bash
   ./dp_lt_model
   ```
   Expected ending: `116/116 tests passed` followed by the coverage report with `Overall: 100.0%`.

### Linux / WSL

```bash
sudo apt-get update && sudo apt-get install -y build-essential git
git clone https://github.com/ABHISHEKSOUNDALGEKAR/DP-link-training-model.git
cd DP-link-training-model
make
./dp_lt_model
```

### Commands

| Command | What it does |
|---|---|
| `./dp_lt_model` | 16 directed tests + 100 random seeds, then the coverage report |
| `./dp_lt_model --list` | List the directed tests |
| `./dp_lt_model <test> -v` | Run one test and print the decoded AUX transaction log |
| `./dp_lt_model --random 1000 --seed 42` | Run 1,000 constrained-random tests starting at seed 42 |
| `make asan` | Rebuild with AddressSanitizer + UBSan and run all tests |
| `make clean` | Remove build output |

The program exits with code 0 only when every test passes, so it can gate CI.

---

## Architecture

```
 SourceTrainer (DP TX)  ──AUX requests──►  AuxChannel  ──►  SinkModel (DP RX, DPCD)
                                               │                 └─ error injection:
                                               │                    DEFER, timeout, CR loss,
                                               │                    slow lock, bad channel
                                               ▼
                                      observers (passive)
                                       ├─ LinkMonitor: protocol checks + coverage
                                       └─ TransactionLogger: decoded AUX log (-v)

 main.cpp: test cases ─► run ─► scoreboard (result vs reference model) + checker results
```

| File | Role | Verification analogy |
|---|---|---|
| `src/dpcd.h/.cpp` | DPCD addresses, bit-field encode/decode, link-rate codes | Register model |
| `src/aux_channel.h/.cpp` | Native AUX read/write, ACK/NACK/DEFER/timeout, simulated time, observer fan-out | Interface + transport |
| `src/sink_model.h/.cpp` | Sink: DPCD space, lane status, adjust requests, error injection | Responder BFM |
| `src/source_trainer.h/.cpp` | Source: capability read, clock recovery, channel EQ, rate/lane fallback, AUX retries | Active BFM / DUT |
| `src/link_monitor.h/.cpp` | Rebuilds training state **only from AUX traffic**, runs checks, samples coverage | Monitor + checker + coverage collector |
| `src/main.cpp` | Test cases, reference model, scoreboard, random generator, runner | Tests + scoreboard |

Design choices:

- **The monitor never reads the source's internal state.** A real VIP only sees the
  interface, so checks and coverage are built from observed transactions.
- **Every checker has a negative test.** A deliberately buggy source must trigger
  exactly the expected rule; otherwise the test fails.
- **The scoreboard is independent of the source.** A reference model predicts the best
  trainable configuration from the channel description alone.
- **Only clean, passing tests contribute coverage.**

### Physical channel abstraction

Each lane at each link rate has two requirements: the voltage swing needed for clock
recovery and the pre-emphasis needed for equalization. If swing + pre-emphasis would
exceed level 3, that lane cannot train at that rate and the source must fall back.
During clock recovery the sink raises its swing request one level per iteration;
during equalization it requests the needed pre-emphasis.

---

## Link-training flow

1. **Capabilities:** read DPCD `0x00000..0x0000F` (max rate, max lanes, TPS3/TPS4
   support, EQ read interval).
2. **Configure:** write `LINK_BW_SET` / `LANE_COUNT_SET` with the highest common configuration.
3. **Clock recovery:** write `TPS1` (scrambling off) plus per-lane drive levels; wait
   100 µs; read `0x00202..0x00207`; apply `ADJUST_REQUEST`; repeat until every lane
   reports `CR_DONE`. Give up after 5 tries at the same swing or when every lane is at max swing.
4. **Channel equalization:** `TPS4` at HBR3 (scrambled), otherwise `TPS3` or `TPS2`;
   wait the EQ read interval; check `CHANNEL_EQ_DONE`, `SYMBOL_LOCKED` and
   `INTERLANE_ALIGN_DONE`; apply adjust requests; up to 5 loops. If `CR_DONE` drops,
   abandon this configuration.
5. **Fallback:** HBR3 → HBR2 → HBR → RBR; after RBR fails, halve the lane count and
   start again from the highest rate.
6. **Done:** write `TRAINING_PATTERN_SET = 0`.

AUX requests that get `DEFER` or no reply are retried up to 7 times; `NACK` is final.

### DPCD registers used

| Address | Name | Use |
|---|---|---|
| `0x00000` | `DPCD_REV` | Capability block start |
| `0x00001` | `MAX_LINK_RATE` | `0x06` RBR, `0x0A` HBR, `0x14` HBR2, `0x1E` HBR3 |
| `0x00002` | `MAX_LANE_COUNT` | `[4:0]` lanes, `[6]` TPS3, `[7]` enhanced framing |
| `0x00003` | `MAX_DOWNSPREAD` | `[7]` TPS4 supported |
| `0x0000E` | `TRAINING_AUX_RD_INTERVAL` | EQ polling interval (0 → 400 µs, n → n × 4 ms) |
| `0x00100` | `LINK_BW_SET` | Selected rate |
| `0x00101` | `LANE_COUNT_SET` | Selected lanes, `[7]` enhanced framing |
| `0x00102` | `TRAINING_PATTERN_SET` | `[3:0]` pattern, `[5]` scrambling disable |
| `0x00103..0x00106` | `TRAINING_LANEx_SET` | Swing `[1:0]`, pre-emphasis `[4:3]`, max-reached flags |
| `0x00202..0x00203` | `LANEx_y_STATUS` | `CR_DONE`, `CHANNEL_EQ_DONE`, `SYMBOL_LOCKED` per lane |
| `0x00204` | `LANE_ALIGN_STATUS_UPDATED` | `[0]` `INTERLANE_ALIGN_DONE` |
| `0x00206..0x00207` | `ADJUST_REQUEST_LANEx_y` | Sink-requested swing and pre-emphasis |

---

## Protocol checks

Implemented in `LinkMonitor`, from observed AUX traffic only.

| Rule | Check | Negative test |
|---|---|---|
| AUX-1 | AUX request length must be 1..16 bytes | — |
| LT-1 | Voltage swing level + pre-emphasis level ≤ 3 on every lane | `neg_illegal_drive` |
| LT-2 | `LINK_BW_SET` is a legal code and ≤ sink `MAX_LINK_RATE` | — |
| LT-3 | `LANE_COUNT_SET` is 1, 2 or 4 and ≤ sink `MAX_LANE_COUNT` | — |
| LT-4 | EQ patterns only after every lane reached `CR_DONE` | — |
| LT-5 | HBR3 equalization uses TPS4 when the sink supports it | `neg_hbr3_without_tps4` |
| LT-6 | Status is polled only after the read interval (CR 100 µs, EQ per DPCD) | `neg_no_read_interval` |
| LT-7 | Training pattern is not left enabled at end of test | — |

---

## Functional coverage

| Coverpoint | Bins |
|---|---|
| `final_rate_x_lanes` | {RBR, HBR, HBR2, HBR3} × {x1, x2, x4} (12 bins) |
| `cr_iterations` | 1, 2–4, 5+ status polls |
| `eq_iterations` | 1, 2+ status polls |
| `eq_pattern` | TPS2, TPS3, TPS4 |
| `fallback` | rate, lane count |
| `cr_lost_during_eq` | hit |
| `aux_defer_streak` | 1, 2–3, 4+ consecutive DEFERs |
| `aux_timeout` | hit |

The default run reaches **100%** (27/27 bins). An earlier version of the random
generator never produced RBR or single-lane links; the coverage report exposed the
hole, and it was closed with the `single_lane_rbr` and `rbr_only_channel` directed
tests plus a wider random distribution of sink rates and lane counts.

---

## Tests

### Directed

| Test | Scenario | Expected |
|---|---|---|
| `happy_hbr3_x4` | Clean channel | HBR3 x4, first try |
| `cr_adjust_requests` | HBR3 needs swing 2 for CR, pre-emphasis 1 for EQ | HBR3 x4 after adjust loops |
| `rate_fallback_lane2` | Lane 2 impossible at HBR3 | HBR2 x4 |
| `lane_fallback` | Lane 3 impossible at every rate | HBR3 x2 |
| `sink_hbr2_x2` | Sink caps limited to HBR2 x2 | HBR2 x2 |
| `aux_defer_storm` | 5 DEFERs | HBR3 x4 after retries |
| `aux_timeouts` | 2 missing replies | HBR3 x4 after retries |
| `aux_retry_exhausted` | 9 DEFERs in a row | Clean failure, no link |
| `cr_lost_during_eq` | Lane 0 drops CR during EQ once | HBR2 x4 |
| `slow_cdr_lock` | CR_DONE withheld for 4 polls | HBR3 x4 after 5 polls |
| `eq_slow_interval` | 4 ms EQ read interval | HBR3 x4, interval honored |
| `single_lane_rbr` | Sink supports only RBR x1 | RBR x1 |
| `rbr_only_channel` | Lane 1 impossible above RBR | RBR x4 |
| `neg_illegal_drive` | Buggy source starts at swing 2 + pre-emphasis 2 | Checker fires LT-1 |
| `neg_no_read_interval` | Buggy source skips the read interval | Checker fires LT-6 |
| `neg_hbr3_without_tps4` | Source without TPS4 at HBR3 | Checker fires LT-5, HBR2 x4 |

### Constrained random

Each seed randomizes the sink's maximum rate (biased toward HBR3) and lane count
(biased toward x4), per-lane channel requirements that grow harder at higher rates,
TPS3 support, the EQ read interval, DEFER and timeout bursts, and CR loss during EQ.
Each result is compared with the reference model, and any checker violation fails the test.

---

## Reading the log

```
$ ./dp_lt_model rate_fallback_lane2 -v
  [      0 us] AUX RD 0x00000 len=16 -> ACK     caps: rev 1.4, max HBR3, 4 lanes, TPS4 yes
  [    228 us] AUX WR 0x00100 len=2  -> ACK     set HBR3 x4
  [    344 us] AUX WR 0x00102 len=5  -> ACK     TPS1 (scrambling off) L0:VS0/PE0 L1:VS0/PE0 L2:VS0/PE0 L3:VS0/PE0
  [    584 us] AUX RD 0x00202 len=6  -> ACK     status L0:CR L1:CR L2:-- L3:CR | adjust req 0/0 0/0 1/0 0/0
  ...
  [   1724 us] AUX RD 0x00202 len=6  -> ACK     status L0:CR L1:CR L2:CR L3:CR | adjust req 0/0 0/0 3/0 0/0
  [   1872 us] AUX WR 0x00102 len=5  -> ACK     TPS4 L0:VS0/PE0 L1:VS0/PE0 L2:VS3/PE0 L3:VS0/PE0
  [   2412 us] AUX RD 0x00202 len=6  -> ACK     status L0:CR+EQ L1:CR+EQ L2:CR L3:CR+EQ | adjust req 0/0 0/0 3/0 0/0
  ...
  [   5412 us] AUX WR 0x00100 len=2  -> ACK     set HBR2 x4
  [   5528 us] AUX WR 0x00102 len=5  -> ACK     TPS1 (scrambling off) L0:VS0/PE0 L1:VS0/PE0 L2:VS0/PE0 L3:VS0/PE0
  [   5768 us] AUX RD 0x00202 len=6  -> ACK     status L0:CR L1:CR L2:CR L3:CR | adjust req 0/0 0/0 0/0 0/0
  [   5916 us] AUX WR 0x00102 len=5  -> ACK     TPS3 (scrambling off) L0:VS0/PE0 L1:VS0/PE0 L2:VS0/PE0 L3:VS0/PE0
  [   6456 us] AUX RD 0x00202 len=6  -> ACK     status L0:CR+EQ L1:CR+EQ L2:CR+EQ L3:CR+EQ ALIGNED | adjust req 0/0 0/0 0/0 0/0
  [   6604 us] AUX WR 0x00102 len=1  -> ACK     NONE
PASS  rate_fallback_lane2      HBR2 x4      6.71 ms
```

Each line shows simulated time, direction, DPCD address, payload length, the sink's
reply, and a decode. Status reads show per-lane `CR` / `CR+EQ`, `ALIGNED`, and the
sink's adjust request per lane as `swing/pre-emphasis`.

In this run, lane 2 needs swing 3 and pre-emphasis 2 at HBR3. That combination is
illegal (sum > 3), so EQ can never complete. After 5 EQ loops the source drops to
HBR2, where the channel trains on the first try.

---

## Simplifications

- The analog channel is a lookup table (minimum swing for CR, minimum pre-emphasis for
  EQ). No eye diagrams, jitter, or real equalizer behavior.
- Native AUX only: no I2C-over-AUX (EDID) and no Manchester-II bit-level timing.
  Transaction durations are approximate.
- 8b/10b rates only: no 128b/132b / UHBR, LTTPR, MST, or post-link-training adjust.
- No HPD or IRQ_HPD; retraining after link loss is not modeled.

## Roadmap

1. HPD and IRQ_HPD: drop lock after training and check that the source retrains.
2. I2C-over-AUX with an EDID read.
3. LTTPR (repeater) support with per-hop training.
4. 128b/132b link training for UHBR rates.
5. A check that the source never retries the same configuration indefinitely.
