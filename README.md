# dp-suspension-controller

> Dynamic Programming controller for active vehicle suspension — a Q1 simulation study targeting **IEEE Open Journal of Vehicular Technology (OJVT)**.

[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/tests-pytest-green)](https://docs.pytest.org/)

This repository implements the simulation plant, baselines, road library, KPI
evaluation, and (in later sprints) the novel Dynamic Programming (DP) controller
for active suspension. The novelty is anchored in handling **nonlinear
constraints (tire liftoff + actuator saturation)** that defeat linear
controllers (LQR) and locally-optimal MPC.

## Sprint status

| Sprint | Goal                                       | Status |
|--------|--------------------------------------------|--------|
| 1      | System modeling & validation (this repo)   | ✅ done |
| 2      | Baseline replication (LQR + MPC-LPV)       | ✅ done |
| 3      | Novel DP controller                        | ⏳ planned |
| 4      | Road excitation library (+ MPC preview)    | ⏳ planned |
| 5      | Evaluation & statistical validation        | ⏳ planned |
| 6      | Reproducibility & journal prep             | ⏳ planned |

## Sprint 2 results (Passive / LQR-tuned / MPC-LPV)

| ISO | v [m/s] | Controller   | RMS a_s [m/s²] | Reduction vs Passive |
|-----|---------|--------------|-----------------|----------------------|
| B   | 20      | Passive      | 0.732           | —                    |
| B   | 20      | LQR-tuned    | 0.241           | **67.0%**            |
| B   | 20      | MPC-LPV      | 0.732           | 0% (no preview)      |
| C   | 20      | Passive      | 1.463           | —                    |
| C   | 20      | LQR-tuned    | 0.574           | **60.8%**            |
| C   | 20      | MPC-LPV      | 1.463           | 0% (no preview)      |
| D   | 20      | Passive      | 2.838           | —                    |
| D   | 20      | LQR-tuned    | 1.547           | **45.5%**            |
| D   | 20      | MPC-LPV      | 2.838           | 0% (no preview)      |

LQR achieves 45–67% RMS body acceleration reduction vs passive on ISO B/C/D
@ 20 m/s. Trade-off: tire load and rattle space worsen — LQR aggressively
suppresses body motion at the cost of wheel hop. On ISO D the LQR saturates
7.6% of the time and loses tire contact 23.5% of the time — exactly the
regime where the DP controller (Sprint 3) should outperform.

MPC-LPV (no road preview) behaves like passive; constraint enforcement works
(0% saturation, 0% rattle violation). Sprint 4 will add road preview.

## Quickstart

```bash
# 1. Create environment
make env                  # uses conda + environment.yml
# or
python -m venv .venv && source .venv/bin/activate && pip install -e ".[dev]"

# 2. Run the validation suite (Sprint 1 gate check)
make test

# 3. Reproduce all figures from raw data (Sprint 6 — stub for now)
make reproduce
```

## Repository layout

```
dp-suspension-controller/
├── configs/                 # YAML configs — single source of truth for params
│   ├── vehicle_quarter_car.yaml
│   └── test_cases.yaml
├── src/dpsus/
│   ├── plant/               # quarter-car 2-DOF, actuator lag, tire liftoff
│   ├── controllers/         # passive, LQR, (later: MPC-LPV, DP)
│   ├── roads/               # ISO 8608 generators (Sprint 4)
│   └── evaluation/          # ISO 2631 KPIs, statistics, plots
├── tests/                   # pytest suite — Sprint 1 gate
├── scripts/                 # one-off validation & reproduction scripts
├── docs/                    # literature review, design notes
├── results/                 # generated figures/tables (gitignored)
├── decision_log.md          # every modeling assumption, dated
├── datasheet.md             # dataset datasheet (Sprint 6)
├── pyproject.toml
├── environment.yml
├── Makefile
└── README.md
```

## Design principles (Golden Rules)

1. **AI writes code, human owns physics.** Never let the AI pick the baseline
   paper or define the stage-cost weights without manual verification.
2. **Lock-in validation gates.** Do not proceed past Sprint 1 until `make test`
   passes with <1% RMS error vs analytical solutions.
3. **Config-driven architecture.** All physical parameters live in
   `configs/*.yaml`. No magic numbers in source code.
4. **Log everything.** Every modeling assumption goes into `decision_log.md`
   with a date and a rationale. This is gold during peer review.

## Target venue

**IEEE Open Journal of Vehicular Technology (OJVT)** — gold open access,
CC-BY, Scopus/SCIE-indexed, ~6–8 week review, no page charges. Recent
vehicle-dynamics papers in OJVT run 10–13 pp with 6–10 figures; Code Ocean
capsules are recommended but not mandated.

## License

MIT — see [LICENSE](LICENSE). Processed road profiles and DP LUTs will be
released under CC-BY 4.0 on Zenodo at submission (Sprint 6).