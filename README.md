# Online EV Routing: Distance-Oriented vs Delay-Aware Rollout

[![Python 3.8+](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Companion code for the paper:

> **Online Electric Vehicle Routing under Time-Dependent Traffic and Charging Delay: A Controlled Comparison of Distance-Oriented and Time-Aware Policies**  
> Trung Duc Tran, Nguyen Toan Thang, Doanh Nguyen Ngoc  
> Center for Environmental Intelligence, VinUniversity, Hanoi, Vietnam

---

## Overview

This repository provides two self-contained Python implementations of online EV routing policies, compared under a shared simulator and Iterated Local Search (ILS) framework:

| Algorithm | File | Decision rule | ILS objective |
|---|---|---|---|
| **Alg 1** – Distance-Oriented | `alg1_distance_oriented.py` | Follow customer plan; repair by nearest reachable station | Traveled distance (km) |
| **Alg 2** – Delay-Aware Rollout | `alg2_delay_aware_rollout.py` | Choose next move by predicted remaining completion time | Total completion time (min) |

The central finding: **shorter distance ≠ shorter completion time** when travel speed is time-varying and charging stations have congestion-dependent waiting delays.

---

## Repository Structure

```
.
├── alg1_distance_oriented.py   # Algorithm 1: distance-oriented baseline + ILS
├── alg2_delay_aware_rollout.py # Algorithm 2: delay-aware rollout policy + ILS
├── generate_data.py            # Standalone script to regenerate all benchmark datasets
├── data/                       # Pre-generated benchmark CSVs (nodes + congested edges)
│   ├── nodes_synth20_U.csv
│   ├── congested_edges_synth20_U.csv
│   └── ... (16 files total, 8 datasets × 2 CSV types)
└── README.md
```

---

## Problem Setting

A single electric vehicle must serve all customers and return to the depot, subject to:

- **Time-dependent travel speed** on congested arcs (3 regimes):
  - Peak `07:00–09:00`, `16:30–19:00` → 20 km/h
  - Normal `09:00–16:30`, `19:00–22:00` → 40 km/h
  - Off-peak `22:00–05:30` → 60 km/h
- **Time-dependent charging delay** at stations (active `15:00–21:30`, triangular profile peaking at `18:00`)
- **Battery constraints**: capacity 30 kWh, safety reserve 3 kWh, consumption 0.2 kWh/km, full recharge at 50 kW chargers
- **Online execution**: charging detours are inserted dynamically; the route is not fixed in advance

The objective is to minimize **realized completion time** T(r).

---

## Quickstart

No external libraries required — pure Python standard library only.

### 1. Clone and run immediately

```bash
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>

# Run Algorithm 1 (fast, ~seconds)
python alg1_distance_oriented.py --sizes 20 40 --dists U M --seeds 1 2 3 --budget quick

# Run Algorithm 2 (slower due to rollout simulation)
python alg2_delay_aware_rollout.py --sizes 20 40 --dists U M --seeds 1 2 3 --budget quick
```

Results are written to `outputs_two_big_files/` by default.

### 2. Regenerate benchmark datasets

The algorithms generate instances on-the-fly from a fixed seed, but you can also pre-generate and inspect all 8 benchmark CSV files:

```bash
python generate_data.py --sizes 20 40 60 80 --dists U M --outdir data
```

This writes `nodes_<dataset>.csv` and `congested_edges_<dataset>.csv` for each of the 8 synthetic datasets.

---

## Command-Line Reference

### `alg1_distance_oriented.py`

```
python alg1_distance_oriented.py [OPTIONS]

Options:
  --sizes INT [INT ...]    Customer counts to test (default: 20 40)
                           Paper uses: 20 40 60 80
  --dists {U,M} [...]      U = uniform random, M = multi-cluster (default: U M)
  --seeds INT [INT ...]    ILS search seeds (default: 1 2 3)
  --data-seed INT          Seed for instance generation (default: 1234)
  --budget {quick,paper}   quick: fast smoke test; paper: full ILS budget (default: quick)
  --outdir PATH            Output folder (default: outputs_two_big_files)
```

### `alg2_delay_aware_rollout.py`

```
python alg2_delay_aware_rollout.py [OPTIONS]

Options:
  --sizes INT [INT ...]    Customer counts to test (default: 20 40)
  --dists {U,M} [...]      U = uniform random, M = multi-cluster (default: U M)
  --seeds INT [INT ...]    ILS search seeds (default: 1 2 3)
  --data-seed INT          Seed for instance generation (default: 1234)
  --budget {quick,paper}   quick: fast smoke test; paper: full ILS budget (default: quick)
  --window INT             Rollout window W; default = round(0.30 × N)
  --outdir PATH            Output folder (default: outputs_two_big_files)
```

### `generate_data.py`

```
python generate_data.py [OPTIONS]

Options:
  --sizes INT [INT ...]    Customer counts (default: 20 40 60 80)
  --dists {U,M} [...]      Distributions (default: U M)
  --data-seed INT          RNG seed (default: 1234)
  --outdir PATH            Output folder (default: data)
```

---

## Reproducing Paper Results

### Quick smoke test (seconds)

```bash
python alg1_distance_oriented.py --sizes 20 --dists U M --seeds 1 2 3 --budget quick
python alg2_delay_aware_rollout.py --sizes 20 --dists U M --seeds 1 2 3 --budget quick
```

### Full paper budget — Algorithm 1 (all 8 datasets, ~5 min)

```bash
python alg1_distance_oriented.py \
    --sizes 20 40 60 80 \
    --dists U M \
    --seeds 1 2 3 \
    --budget paper \
    --outdir results_paper
```

### Full paper budget — Algorithm 2 (all 8 datasets, ~hours)

> ⚠️ Algorithm 2 is computationally intensive. For N=80 the rollout simulator is called many times per ILS iteration. Expected runtime on a mid-range laptop: ~30 min (N≤40) to several hours (N=80, `--budget paper`).

```bash
# Recommended: run one size at a time
python alg2_delay_aware_rollout.py \
    --sizes 20 40 \
    --dists U M \
    --seeds 1 2 3 \
    --budget paper \
    --outdir results_paper

python alg2_delay_aware_rollout.py \
    --sizes 60 80 \
    --dists U M \
    --seeds 1 2 3 \
    --budget paper \
    --outdir results_paper
```

### Replicating the tuned W* sweep (Table 3)

To sweep the rollout window as in the paper (W/N ∈ {0.1, 0.2, 0.3, 0.4, 0.5}):

```bash
for W_FRAC in 0.1 0.2 0.3 0.4 0.5; do
  N=20
  W=$(python -c "import math; print(max(2, round($W_FRAC * $N)))")
  python alg2_delay_aware_rollout.py \
      --sizes $N --dists U M --seeds 1 \
      --budget paper --window $W \
      --outdir sweep_N${N}_W${W}
done
```

---

## Output Files

Each run writes the following to the specified `--outdir`:

| File | Contents |
|---|---|
| `results_alg1.csv` | Per-run metrics: completion time, distance, drive/wait/charge/service breakdown, runtime |
| `routes_alg1.csv` | Customer plan and full executed route sequence for each run |
| `results_alg2.csv` | Same as above for Algorithm 2, plus rollout window W and candidate size K |
| `routes_alg2.csv` | Same as above for Algorithm 2 |
| `nodes_<dataset>.csv` | Node coordinates and station wait parameters |
| `congested_edges_<dataset>.csv` | Edge pairs flagged as congested, with distance |

### `results_alg1.csv` column reference

| Column | Description |
|---|---|
| `dataset` | Instance name, e.g. `synth40_M` |
| `N` | Number of customers |
| `distribution` | `U` (uniform) or `M` (multi-cluster) |
| `seed` | ILS search seed |
| `algorithm` | `alg1_distance_oriented` |
| `objective_used_by_ILS` | `distance_km` for Alg1, `total_time_min` for Alg2 |
| `feasible` | Whether the executed route satisfies all battery constraints |
| `total_time_min` | Realized completion time T(r) in minutes |
| `distance_km` | Total traveled distance D(r) in km |
| `drive_min` | Total drive time component |
| `wait_min` | Total station waiting time component |
| `charge_min` | Total charging time component |
| `service_min` | Total customer service time component |
| `charges` | Number of charging stops |
| `route_len` | Number of nodes in the executed route |
| `runtime_sec` | Wall-clock time for this run in seconds |

---

## Synthetic Benchmark Datasets

Eight datasets are used, spanning two spatial layouts × four customer counts:

| Dataset name | N | Layout | Charging stations |
|---|---|---|---|
| `synth20_U` | 20 | Uniform random | 2 |
| `synth20_M` | 20 | Multi-cluster (5 centers) | 2 |
| `synth40_U` | 40 | Uniform random | 4 |
| `synth40_M` | 40 | Multi-cluster | 4 |
| `synth60_U` | 60 | Uniform random | 6 |
| `synth60_M` | 60 | Multi-cluster | 6 |
| `synth80_U` | 80 | Uniform random | 8 |
| `synth80_M` | 80 | Multi-cluster | 8 |

Stations are fixed at 10% of customer count, placed at well-spread coordinates to prevent unreachable islands. Station waiting parameters are drawn from Uniform(10, 35) minutes (base wait at peak congestion, 18:00).

---

## Algorithm Details

### Algorithm 1 – Distance-Oriented Evaluation + ILS

```
Given customer plan π:
  while unserved customers remain:
    try to serve next customer in π
    if unsafe → detour to nearest reachable station (by distance)
    if already at station and unsafe → charge to full
  optimize π with ILS using D(r) as objective
```

Station choice is **purely geometric** — waiting delay and downstream timing are not considered.

### Algorithm 2 – Delay-Aware Rollout + ILS

```
Given customer plan π and rollout window W:
  while unserved customers remain:
    form candidate set from first W customers (filtered by travel time to top K)
    for each candidate c:
      estimate remaining completion time by forward simulation
    select c* = argmin predicted completion time
    if c* is unsafe → evaluate all reachable stations by:
      travel_time + wait_time + charge_time + downstream_completion_time
      select best station
  optimize π with ILS using T(r) as objective
```

Station choice is **delay-aware** — the score of each candidate station includes the downstream simulated continuation time from the post-charge state.

### Shared ILS Framework

Both algorithms share:
- Nearest-neighbor depot-anchored initial customer plan
- Same 2-opt / swap / relocate neighborhood operators
- Simulated-annealing acceptance with temperature-based non-improving moves
- Adaptive budgets scaled by instance size (see `get_budget()` in each file)

---

## Key Results (from the paper)

**Fixed-setting three-seed study** (Table 4 & 5 in the paper):

| Metric | Value |
|---|---|
| Paired runs compared | 24 (8 datasets × 3 seeds) |
| Runs where Alg2 < Alg1 | **24 / 24** |
| Mean ΔT (Alg2 − Alg1) | **−42.46 min** |
| Bootstrap 95% CI for mean ΔT | [−55.23, −31.97] min |
| Sign test p-value | 5.96 × 10⁻⁸ |

**Runtime trade-off** (mean across 24 runs):

| Algorithm | Mean runtime |
|---|---|
| Alg1 | ~0.21 seconds |
| Alg2 | ~105.58 seconds |

> The time savings of Alg2 come at a substantial computational cost. In 5 of 8 datasets, Alg2 travels **farther** in km while still finishing **earlier** in time — consistent with the paper's conclusion that geometric distance is not a reliable proxy for completion time under time-varying traffic and charging delay.

---

