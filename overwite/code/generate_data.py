
from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Set, Tuple

DEPOT_ID          = 0
ENERGY_RATE       = 0.2      # kWh / km
CONGESTED_DENSITY = 0.80

@dataclass(frozen=True)
class Node:
    node_id: int
    kind: str          # depot | customer | station
    x: float
    y: float
    wait_base_min: float = 0.0


def euclidean(a: Node, b: Node) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def clamp(v: float, lo: float = 2.0, hi: float = 98.0) -> float:
    return max(lo, min(hi, v))


def station_coordinates(m: int) -> List[Tuple[float, float]]:
    base = [
        (35, 50), (65, 50), (50, 35), (50, 65),
        (25, 25), (75, 25), (25, 75), (75, 75),
        (50, 15), (50, 85), (15, 50), (85, 50),
    ]
    if m <= len(base):
        return base[:m]
    coords = base[:]
    side = math.ceil(math.sqrt(m - len(base)))
    for r in range(side):
        for c in range(side):
            if len(coords) >= m:
                break
            coords.append((10 + 80*c/max(1,side-1), 10 + 80*r/max(1,side-1)))
        if len(coords) >= m:
            break
    return coords[:m]


def generate_instance(n_customers: int, distribution: str, seed: int = 1234):
    rng  = random.Random(seed + n_customers*100 + (0 if distribution.upper()=="U" else 1))
    dist = distribution.upper()
    name = f"synth{n_customers}_{dist}"
    nodes: Dict[int, Node] = {DEPOT_ID: Node(DEPOT_ID, "depot", 50.0, 50.0, 0.0)}

    if dist == "U":
        for cid in range(1, n_customers+1):
            nodes[cid] = Node(cid, "customer", rng.uniform(5,95), rng.uniform(5,95), 0.0)
    elif dist == "M":
        centers = [(25,25),(75,25),(30,75),(75,75),(50,50)]
        for cid in range(1, n_customers+1):
            cx, cy = rng.choice(centers)
            nodes[cid] = Node(cid,"customer", clamp(rng.gauss(cx,8.0)), clamp(rng.gauss(cy,8.0)), 0.0)
    else:
        raise ValueError("distribution must be U or M")

    m_st = max(1, round(0.10 * n_customers))
    for k, (x, y) in enumerate(station_coordinates(m_st), start=n_customers+1):
        nodes[k] = Node(k, "station", x, y, rng.uniform(10.0, 35.0))

    ids = sorted(nodes)
    distances: Dict[Tuple[int,int], float] = {}
    for i in ids:
        for j in ids:
            distances[(i,j)] = 0.0 if i==j else euclidean(nodes[i], nodes[j])

    pairs = [(i,j) for idx,i in enumerate(ids) for j in ids[idx+1:]]
    rng.shuffle(pairs)
    k_cong = int(CONGESTED_DENSITY * len(pairs))
    congested: Set[Tuple[int,int]] = set(tuple(sorted(p)) for p in pairs[:k_cong])

    return name, nodes, distances, congested, \
           list(range(1, n_customers+1)), \
           list(range(n_customers+1, n_customers+1+m_st))


def save(name, nodes, distances, congested, outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)

    # nodes
    with (outdir / f"nodes_{name}.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dataset","node_id","kind","x","y","wait_base_min"])
        for nid in sorted(nodes):
            nd = nodes[nid]
            w.writerow([name, nd.node_id, nd.kind,
                        f"{nd.x:.4f}", f"{nd.y:.4f}", f"{nd.wait_base_min:.4f}"])

    # congested edges
    with (outdir / f"congested_edges_{name}.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dataset","i","j","distance_km"])
        for i,j in sorted(congested):
            w.writerow([name, i, j, f"{distances[(i,j)]:.4f}"])

    print(f"  Saved nodes_{name}.csv  and  congested_edges_{name}.csv")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--sizes", type=int, nargs="+", default=[20,40,60,80])
    p.add_argument("--dists", type=str, nargs="+", default=["U","M"])
    p.add_argument("--data-seed", type=int, default=1234)
    p.add_argument("--outdir", type=str, default="data")
    args = p.parse_args()

    out = Path(args.outdir)
    print(f"Generating datasets → {out}/")
    for n in args.sizes:
        for d in args.dists:
            name, nodes, dist, cong, *_ = generate_instance(n, d, args.data_seed)
            save(name, nodes, dist, cong, out)
    print(f"\nDone. {len(args.sizes)*len(args.dists)*2} CSV files written to '{out}/'.")


if __name__ == "__main__":
    main()
