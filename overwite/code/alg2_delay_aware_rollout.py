#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import csv
import math
import os
import random
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except ImportError:
    pywrapcp = None
    routing_enums_pb2 = None



DEPOT_ID = 0
SERVICE_MIN = 5.0
START_TIME_MIN = 7 * 60.0
BATTERY_CAPACITY_KWH = 30.0
RESERVE_KWH = 3.0
ENERGY_RATE_KWH_PER_KM = 0.2
CHARGER_POWER_KW = 50.0
CHARGE_RATE_KWH_PER_MIN = CHARGER_POWER_KW / 60.0
CONGESTED_EDGE_DENSITY = 0.80
INFEASIBLE = 1e12
EPS = 1e-9


@dataclass(frozen=True)
class Node:
    node_id: int
    kind: str
    x: float
    y: float
    wait_base_min: float = 0.0


@dataclass
class Instance:
    name: str
    n_customers: int
    distribution: str
    nodes: Dict[int, Node]
    customers: List[int]
    stations: List[int]
    dist: Dict[Tuple[int, int], float]
    congested_pairs: Set[Tuple[int, int]]

    def d(self, i: int, j: int) -> float:
        return self.dist[(i, j)]

    def energy(self, i: int, j: int) -> float:
        return ENERGY_RATE_KWH_PER_KM * self.d(i, j)

    def is_station(self, node_id: int) -> bool:
        return self.nodes[node_id].kind == "station"

    def is_customer(self, node_id: int) -> bool:
        return self.nodes[node_id].kind == "customer"

    def is_congested(self, i: int, j: int) -> bool:
        a, b = sorted((i, j))
        return (a, b) in self.congested_pairs

    def travel_speed_kmph(self, i: int, j: int, depart_min: float) -> float:
        if not self.is_congested(i, j):
            return 60.0
        m = depart_min % 1440.0
        if (420 <= m < 540) or (990 <= m < 1140):
            return 20.0
        if (330 <= m < 420) or (540 <= m < 990) or (1140 <= m < 1320):
            return 40.0
        return 60.0

    def travel_time_min(self, i: int, j: int, depart_min: float) -> float:
        speed = self.travel_speed_kmph(i, j, depart_min)
        return self.d(i, j) / speed * 60.0

    def station_wait_min(self, station_id: int, arrival_min: float) -> float:
        if not self.is_station(station_id):
            return 0.0
        m = arrival_min % 1440.0
        if not (900 <= m < 1290):
            return 0.0
        base = self.nodes[station_id].wait_base_min
        center = 18 * 60.0
        half_width = 195.0
        shape = max(0.0, 1.0 - abs(m - center) / half_width)
        return base * (0.40 + 0.60 * shape)


@dataclass
class RouteResult:
    feasible: bool
    total_time_min: float = INFEASIBLE
    distance_km: float = INFEASIBLE
    drive_min: float = 0.0
    wait_min: float = 0.0
    charge_min: float = 0.0
    service_min: float = 0.0
    charges: int = 0
    route: List[int] = field(default_factory=list)
    reason: str = ""

    def objective_distance(self) -> float:
        return self.distance_km if self.feasible else INFEASIBLE

    def objective_time(self) -> float:
        return self.total_time_min if self.feasible else INFEASIBLE


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
            coords.append((10 + 80 * c / max(1, side - 1), 10 + 80 * r / max(1, side - 1)))
        if len(coords) >= m:
            break
    return coords[:m]


def generate_instance(n_customers: int, distribution: str, seed: int = 1234) -> Instance:
    rng = random.Random(seed + n_customers * 100 + (0 if distribution.upper() == "U" else 1))
    distribution = distribution.upper()
    name = f"synth{n_customers}_{distribution}"
    nodes: Dict[int, Node] = {DEPOT_ID: Node(DEPOT_ID, "depot", 50.0, 50.0, 0.0)}

    if distribution == "U":
        for cid in range(1, n_customers + 1):
            nodes[cid] = Node(cid, "customer", rng.uniform(5, 95), rng.uniform(5, 95), 0.0)
    elif distribution == "M":
        centers = [(25, 25), (75, 25), (30, 75), (75, 75), (50, 50)]
        for cid in range(1, n_customers + 1):
            cx, cy = rng.choice(centers)
            nodes[cid] = Node(cid, "customer", clamp(rng.gauss(cx, 8.0)), clamp(rng.gauss(cy, 8.0)), 0.0)
    else:
        raise ValueError("distribution must be U or M")

    m_stations = max(1, round(0.10 * n_customers))
    for k, (x, y) in enumerate(station_coordinates(m_stations), start=n_customers + 1):
        wait_base = rng.uniform(10.0, 35.0)
        nodes[k] = Node(k, "station", x, y, wait_base)

    ids = sorted(nodes)
    dist: Dict[Tuple[int, int], float] = {}
    for i in ids:
        for j in ids:
            dist[(i, j)] = 0.0 if i == j else euclidean(nodes[i], nodes[j])

    pairs = [(i, j) for idx, i in enumerate(ids) for j in ids[idx + 1:]]
    rng.shuffle(pairs)
    k_cong = int(CONGESTED_EDGE_DENSITY * len(pairs))
    congested_pairs = set(tuple(sorted(p)) for p in pairs[:k_cong])

    return Instance(
        name=name,
        n_customers=n_customers,
        distribution=distribution,
        nodes=nodes,
        customers=list(range(1, n_customers + 1)),
        stations=list(range(n_customers + 1, n_customers + 1 + m_stations)),
        dist=dist,
        congested_pairs=congested_pairs,
    )


def charge_time_to_full_min(battery_kwh: float) -> float:
    missing = max(0.0, BATTERY_CAPACITY_KWH - battery_kwh)
    return missing / CHARGE_RATE_KWH_PER_MIN


def can_reach(inst: Instance, i: int, j: int, battery_kwh: float) -> bool:
    return battery_kwh + EPS >= inst.energy(i, j) + RESERVE_KWH


def safe_customer_move(inst: Instance, i: int, j: int, battery_kwh: float) -> bool:
    if not can_reach(inst, i, j, battery_kwh):
        return False
    b_after = battery_kwh - inst.energy(i, j)
    targets = inst.stations + [DEPOT_ID]
    return any(b_after + EPS >= inst.energy(j, s) + RESERVE_KWH for s in targets)


def reachable_stations(inst: Instance, current: int, battery_kwh: float, exclude_current: bool = True) -> List[int]:
    out = []
    for s in inst.stations:
        if exclude_current and s == current:
            continue
        if can_reach(inst, current, s, battery_kwh):
            out.append(s)
    return out


def move_to(inst: Instance, result: RouteResult, current: int, target: int,
            now_min: float, battery_kwh: float) -> Tuple[int, float, float, bool]:
    if target == current:
        result.feasible = False
        result.reason = "attempted self move"
        return current, now_min, battery_kwh, False
    if not can_reach(inst, current, target, battery_kwh):
        result.feasible = False
        result.reason = f"battery infeasible: {current}->{target}"
        return current, now_min, battery_kwh, False

    dist = inst.d(current, target)
    travel = inst.travel_time_min(current, target, now_min)
    energy = inst.energy(current, target)
    now_min += travel
    battery_kwh -= energy

    result.distance_km += dist
    result.drive_min += travel
    result.route.append(target)

    if inst.is_customer(target):
        now_min += SERVICE_MIN
        result.service_min += SERVICE_MIN
    elif inst.is_station(target):
        wait = inst.station_wait_min(target, now_min)
        charge = charge_time_to_full_min(battery_kwh)
        now_min += wait + charge
        battery_kwh = BATTERY_CAPACITY_KWH
        result.wait_min += wait
        result.charge_min += charge
        result.charges += 1

    return target, now_min, battery_kwh, True


def init_result() -> RouteResult:
    return RouteResult(feasible=True, total_time_min=0.0, distance_km=0.0, route=[DEPOT_ID])


def nearest_neighbor_customer_plan(inst: Instance) -> List[int]:
    unvisited = set(inst.customers)
    plan: List[int] = []
    current = DEPOT_ID
    while unvisited:
        nxt = min(unvisited, key=lambda c: inst.d(current, c))
        plan.append(nxt)
        unvisited.remove(nxt)
        current = nxt
    return plan


def random_neighbor(plan: List[int], rng: random.Random) -> List[int]:
    n = len(plan)
    if n < 2:
        return plan[:]
    p = plan[:]
    move = rng.choice(["swap", "relocate", "two_opt"])
    i, j = sorted(rng.sample(range(n), 2))
    if move == "swap":
        p[i], p[j] = p[j], p[i]
    elif move == "relocate":
        item = p.pop(j)
        p.insert(i, item)
    else:
        p[i:j + 1] = reversed(p[i:j + 1])
    return p


def perturb(plan: List[int], rng: random.Random, strength: int) -> List[int]:
    p = plan[:]
    for _ in range(max(1, strength)):
        p = random_neighbor(p, rng)
    return p


def solve_customer_plan_ortools(inst: Instance, time_limit_sec: int = 3) -> List[int]:
    if pywrapcp is None or routing_enums_pb2 is None:
        raise RuntimeError("OR-Tools is not installed. Install it with: pip install ortools")

    node_ids = [DEPOT_ID] + inst.customers
    manager = pywrapcp.RoutingIndexManager(len(node_ids), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index: int, to_index: int) -> int:
        i = node_ids[manager.IndexToNode(from_index)]
        j = node_ids[manager.IndexToNode(to_index)]
        return int(round(inst.d(i, j) * 1000))

    transit_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = max(1, int(time_limit_sec))

    solution = routing.SolveWithParameters(params)
    if solution is None:
        return nearest_neighbor_customer_plan(inst)

    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node = node_ids[manager.IndexToNode(index)]
        if node != DEPOT_ID:
            route.append(node)
        index = solution.Value(routing.NextVar(index))
    return route

def approximate_remaining_time(inst: Instance, current0: int, now0: float, battery0: float,
                               plan0: Sequence[int], max_steps: int = 2000) -> float:
    current = current0
    now = now0
    battery = battery0
    plan = list(plan0)
    elapsed = 0.0
    steps = 0

    def best_station_for(next_node: int) -> Optional[int]:
        stations = reachable_stations(inst, current, battery, exclude_current=True)
        if not stations:
            return None
        best_s, best_score = None, INFEASIBLE
        for s in stations:
            travel = inst.travel_time_min(current, s, now)
            arr = now + travel
            b_arr = battery - inst.energy(current, s)
            wait = inst.station_wait_min(s, arr)
            charge = charge_time_to_full_min(b_arr)
            after = arr + wait + charge
            look = inst.travel_time_min(s, next_node, after) if next_node != s else 0.0
            score = travel + wait + charge + look
            if score < best_score:
                best_s, best_score = s, score
        return best_s

    while True:
        steps += 1
        if steps > max_steps:
            return INFEASIBLE

        if plan:
            target = plan[0]
            if safe_customer_move(inst, current, target, battery):
                travel = inst.travel_time_min(current, target, now)
                now += travel + SERVICE_MIN
                elapsed += travel + SERVICE_MIN
                battery -= inst.energy(current, target)
                current = target
                plan.pop(0)
            else:
                s = best_station_for(target)
                if s is None:
                    return INFEASIBLE
                travel = inst.travel_time_min(current, s, now)
                arr = now + travel
                b_arr = battery - inst.energy(current, s)
                wait = inst.station_wait_min(s, arr)
                charge = charge_time_to_full_min(b_arr)
                now = arr + wait + charge
                elapsed += travel + wait + charge
                battery = BATTERY_CAPACITY_KWH
                current = s
        else:
            if current == DEPOT_ID:
                return elapsed
            if can_reach(inst, current, DEPOT_ID, battery):
                travel = inst.travel_time_min(current, DEPOT_ID, now)
                elapsed += travel
                return elapsed
            s = best_station_for(DEPOT_ID)
            if s is None:
                return INFEASIBLE
            travel = inst.travel_time_min(current, s, now)
            arr = now + travel
            b_arr = battery - inst.energy(current, s)
            wait = inst.station_wait_min(s, arr)
            charge = charge_time_to_full_min(b_arr)
            now = arr + wait + charge
            elapsed += travel + wait + charge
            battery = BATTERY_CAPACITY_KWH
            current = s


def choose_rollout_customer(inst: Instance, current: int, now: float, battery: float,
                            plan: List[int], window: int, k_candidates: int) -> int:
    prefix = plan[:max(1, min(window, len(plan)))]
    if k_candidates > 0 and k_candidates < len(prefix):
        prefix = sorted(prefix, key=lambda c: inst.travel_time_min(current, c, now))[:k_candidates]

    best_c, best_score = prefix[0], INFEASIBLE
    for c in prefix:
        tmp = [c] + [x for x in plan if x != c]
        score = approximate_remaining_time(inst, current, now, battery, tmp)
        if score < best_score:
            best_c, best_score = c, score
    return best_c


def choose_delayaware_station(inst: Instance, current: int, now: float, battery: float,
                              plan: Sequence[int]) -> Optional[int]:
    candidates = reachable_stations(inst, current, battery, exclude_current=True)
    if not candidates:
        return None
    best_s, best_score = None, INFEASIBLE
    for s in candidates:
        travel = inst.travel_time_min(current, s, now)
        arr = now + travel
        b_arr = battery - inst.energy(current, s)
        wait = inst.station_wait_min(s, arr)
        charge = charge_time_to_full_min(b_arr)
        after = arr + wait + charge
        downstream = approximate_remaining_time(inst, s, after, BATTERY_CAPACITY_KWH, plan)
        score = travel + wait + charge + downstream
        if score < best_score:
            best_s, best_score = s, score
    return best_s


def execute_rollout_policy(inst: Instance, plan: Sequence[int], window: int,
                           k_candidates: Optional[int] = None, max_steps: int = 10000) -> RouteResult:
    result = init_result()
    current = DEPOT_ID
    now = START_TIME_MIN
    battery = BATTERY_CAPACITY_KWH
    remaining = list(plan)
    window = max(1, int(window))
    if k_candidates is None:
        k_candidates = max(1, window // 2)
    k_candidates = max(1, int(k_candidates))
    steps = 0

    while remaining:
        steps += 1
        if steps > max_steps:
            result.feasible = False
            result.reason = "max_steps exceeded in rollout policy"
            break

        chosen = choose_rollout_customer(inst, current, now, battery, remaining, window, k_candidates)
        if safe_customer_move(inst, current, chosen, battery):
            current, now, battery, ok = move_to(inst, result, current, chosen, now, battery)
            if not ok:
                break
            remaining.remove(chosen)
        else:
            s = choose_delayaware_station(inst, current, now, battery, remaining)
            if s is None:
                result.feasible = False
                result.reason = f"rollout: no reachable station from node {current}"
                break
            current, now, battery, ok = move_to(inst, result, current, s, now, battery)
            if not ok:
                break

    while result.feasible and current != DEPOT_ID:
        steps += 1
        if steps > max_steps:
            result.feasible = False
            result.reason = "max_steps exceeded during rollout depot return"
            break
        if can_reach(inst, current, DEPOT_ID, battery):
            current, now, battery, ok = move_to(inst, result, current, DEPOT_ID, now, battery)
            if not ok:
                break
        else:
            s = choose_delayaware_station(inst, current, now, battery, [])
            if s is None:
                result.feasible = False
                result.reason = f"rollout cannot return: no reachable station from {current}"
                break
            current, now, battery, ok = move_to(inst, result, current, s, now, battery)
            if not ok:
                break

    result.total_time_min = now - START_TIME_MIN if result.feasible else INFEASIBLE
    return result


@dataclass(frozen=True)
class Budget:
    ils_alg1: int
    ils_alg2: int
    ls_rounds: int
    ls_neighbors: int
    perturb_strength: int
    temp0: float = 0.03
    cooling: float = 0.98


def get_budget(n: int, kind: str, alg: str) -> Budget:
    kind = kind.lower()
    if kind == "paper":
        if n <= 20:
            a1, a2, rounds, neigh = 10, 10, 4, 8
        elif n <= 40:
            a1, a2, rounds, neigh = 14, 18, 5, 8
        elif n <= 60:
            a1, a2, rounds, neigh = 21, 27, 7, 12
        else:
            a1, a2, rounds, neigh = 28, 36, 10, 16
    elif kind == "quick":
        if n <= 20:
            a1, a2, rounds, neigh = 4, 5, 2, 4
        elif n <= 40:
            a1, a2, rounds, neigh = 5, 7, 2, 5
        elif n <= 60:
            a1, a2, rounds, neigh = 7, 9, 3, 6
        else:
            a1, a2, rounds, neigh = 9, 12, 3, 8
    else:
        raise ValueError("budget must be quick or paper")
    strength = max(1, round(0.15 * n))
    return Budget(a1, a2, rounds, neigh, strength)


def local_search(plan0: List[int], evaluator: Callable[[List[int]], RouteResult],
                 objective: Callable[[RouteResult], float], rng: random.Random,
                 rounds: int, neighbors_per_round: int) -> Tuple[List[int], RouteResult]:
    current_plan = plan0[:]
    current_result = evaluator(current_plan)
    current_obj = objective(current_result)

    improved = True
    while improved:
        improved = False
        for _ in range(rounds):
            accepted = False
            for _ in range(neighbors_per_round):
                cand_plan = random_neighbor(current_plan, rng)
                cand_result = evaluator(cand_plan)
                cand_obj = objective(cand_result)
                if cand_obj + EPS < current_obj:
                    current_plan, current_result, current_obj = cand_plan, cand_result, cand_obj
                    improved = True
                    accepted = True
                    break
            if accepted:
                break
    return current_plan, current_result


def ils(plan0: List[int], evaluator: Callable[[List[int]], RouteResult],
        objective: Callable[[RouteResult], float], rng: random.Random,
        budget: Budget, outer_iterations: int) -> Tuple[List[int], RouteResult]:
    cur_plan, cur_result = local_search(
        plan0, evaluator, objective, rng, budget.ls_rounds, budget.ls_neighbors
    )
    cur_obj = objective(cur_result)
    best_plan, best_result, best_obj = cur_plan[:], cur_result, cur_obj
    temp = budget.temp0

    for _ in range(outer_iterations):
        trial_start = perturb(cur_plan, rng, budget.perturb_strength)
        trial_plan, trial_result = local_search(
            trial_start, evaluator, objective, rng, budget.ls_rounds, budget.ls_neighbors
        )
        trial_obj = objective(trial_result)
        delta = trial_obj - cur_obj
        accept = delta <= 0
        if not accept and math.isfinite(delta) and cur_obj < INFEASIBLE:
            rel = delta / max(1.0, abs(cur_obj))
            prob = math.exp(-rel / max(1e-9, temp))
            accept = rng.random() < prob
        if accept:
            cur_plan, cur_result, cur_obj = trial_plan, trial_result, trial_obj
        if trial_obj + EPS < best_obj:
            best_plan, best_result, best_obj = trial_plan[:], trial_result, trial_obj
        temp *= budget.cooling

    return best_plan, best_result


def window_for_n(n: int) -> int:
    return max(4, int(round(0.30 * n)))


def run_one(inst: Instance, seed: int, budget_kind: str, window: Optional[int] = None,
            ortools_time_limit: int = 3) -> Tuple[List[int], RouteResult, float]:
    W = window if window is not None else window_for_n(inst.n_customers)
    K = max(1, W // 2)

    start = time.perf_counter()
    plan = solve_customer_plan_ortools(inst, time_limit_sec=ortools_time_limit)
    result = execute_rollout_policy(inst, plan, window=W, k_candidates=K)
    runtime = time.perf_counter() - start
    return plan, result, runtime


def save_instance(inst: Instance, outdir: Path) -> None:
    outdir.mkdir(parents=True, exist_ok=True)
    node_path = outdir / f"nodes_{inst.name}.csv"
    with node_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["dataset", "node_id", "kind", "x", "y", "wait_base_min"])
        for node_id in sorted(inst.nodes):
            nd = inst.nodes[node_id]
            w.writerow([inst.name, nd.node_id, nd.kind, f"{nd.x:.4f}", f"{nd.y:.4f}", f"{nd.wait_base_min:.4f}"])

    edge_path = outdir / f"congested_edges_{inst.name}.csv"
    with edge_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["dataset", "i", "j", "distance_km"])
        for i, j in sorted(inst.congested_pairs):
            w.writerow([inst.name, i, j, f"{inst.d(i, j):.4f}"])


def route_to_string(route: Sequence[int]) -> str:
    return "-".join(str(x) for x in route)


def run_suite(args: argparse.Namespace) -> None:
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    results_rows: List[Dict[str, object]] = []
    route_rows: List[Dict[str, object]] = []

    for n in args.sizes:
        for dist_name in args.dists:
            inst = generate_instance(n, dist_name, seed=args.data_seed)
            save_instance(inst, outdir)
            W = args.window if args.window is not None else window_for_n(n)
            K = max(1, W // 2)

            for search_seed in args.seeds:
                print(f"Running Alg2 on {inst.name}, seed={search_seed}, W={W}, K={K}, budget={args.budget} ...", flush=True)
                plan, res, runtime = run_one(inst, search_seed, args.budget, window=W, ortools_time_limit=args.ortools_time_limit)
                row = {
                    "dataset": inst.name,
                    "N": n,
                    "distribution": dist_name.upper(),
                    "seed": search_seed,
                    "algorithm": "alg2_delay_aware_rollout_ortools",
                    "customer_order_solver": "ortools_routing_distance",
                    "W": W,
                    "K": K,
                    "feasible": res.feasible,
                    "total_time_min": round(res.total_time_min, 4),
                    "distance_km": round(res.distance_km, 4),
                    "drive_min": round(res.drive_min, 4),
                    "wait_min": round(res.wait_min, 4),
                    "charge_min": round(res.charge_min, 4),
                    "service_min": round(res.service_min, 4),
                    "charges": res.charges,
                    "route_len": len(res.route),
                    "runtime_sec": round(runtime, 4),
                    "reason": res.reason,
                }
                results_rows.append(row)
                route_rows.append({
                    "dataset": inst.name,
                    "seed": search_seed,
                    "algorithm": "alg2_delay_aware_rollout_ortools",
                    "W": W,
                    "K": K,
                    "plan": route_to_string(plan),
                    "executed_route": route_to_string(res.route),
                })
                print(
                    f"  Alg2: feasible={res.feasible}, T={res.total_time_min:.2f} min, "
                    f"D={res.distance_km:.2f} km, charges={res.charges}, runtime={runtime:.2f}s",
                    flush=True,
                )

    results_path = outdir / "results_alg2.csv"
    with results_path.open("w", newline="", encoding="utf-8") as f:
        fields = list(results_rows[0].keys()) if results_rows else []
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(results_rows)

    routes_path = outdir / "routes_alg2.csv"
    with routes_path.open("w", newline="", encoding="utf-8") as f:
        fields = list(route_rows[0].keys()) if route_rows else []
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(route_rows)

    print("\nDone Alg2.")
    print(f"Saved: {results_path}")
    print(f"Saved: {routes_path}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Alg2: delay-aware rollout EVRP policy with OR-Tools customer ordering.")
    p.add_argument("--sizes", type=int, nargs="+", default=[20, 40], help="Customer counts, e.g. --sizes 20 40 60 80")
    p.add_argument("--dists", type=str, nargs="+", default=["U", "M"], choices=["U", "M", "u", "m"], help="Spatial distributions: U uniform, M multicluster")
    p.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3], help="ILS search seeds")
    p.add_argument("--data-seed", type=int, default=1234, help="Seed for generated datasets and congestion graph")
    p.add_argument("--budget", type=str, default="quick", choices=["quick", "paper"], help="kept for compatibility")
    p.add_argument("--ortools-time-limit", type=int, default=3, help="OR-Tools search time limit in seconds")
    p.add_argument("--window", type=int, default=None, help="Rollout window W. Default is round(0.30*N).")
    p.add_argument("--outdir", type=str, default="outputs_two_big_files", help="Output folder")
    return p.parse_args()


if __name__ == "__main__":
    run_suite(parse_args())
