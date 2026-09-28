#!/usr/bin/env python3
"""analyze.py RESULTS_DIR: the tables of ../README.md and the decision of ../DECISION_RULE.md,
from RESULTS_DIR/final.jsonl and RESULTS_DIR/growth.jsonl as run_all.sh writes them.

Scores are relative to CoordMap: the harness's (coordmap) for the harness variants, the
branch at d6b4d88 (real_coordmap) for the code built into Bonxai."""
import collections
import json
import math
import os
import statistics
import sys

VG = ["build_ms", "hit_ms", "miss_ms", "update_ms", "iter_ms", "clear_ms"]
VG4 = ["build_ms", "hit_ms", "miss_ms", "iter_ms"]
VG3 = ["build_ms", "hit_ms", "miss_ms"]
MAP = ["insert_ns", "insert_reserved_ns", "hit_ns", "miss_ns", "mix_ns", "churn_ns", "iter_ns", "clear_ns"]
# category, workload, metrics
GROUPS = [
    ("large", "wide", VG), ("large", "wide1m", VG), ("large", "vg_rand_250000", VG4),
    ("large", "widepos", VG3),
    ("keys", "vg_dense_250000", VG3), ("keys", "vg_plane_250000", VG3),
    ("keys", "vg_line_250000", VG3), ("keys", "vg_stride_250000", VG3),
    ("small", "vg_rand_1000", VG4), ("small", "vg_rand_10000", VG4),
    ("small", "room", ["build_ms", "scan_ms", "shuffled_ms", "update_ms"]),
    ("map", "map_rand_1000", MAP), ("map", "map_rand_100000", MAP),
    ("map", "map_rand_1000000", MAP), ("map", "map_dense_100000", MAP[:6]),
    ("e2e", "rays10", ["build_ms", "randq_ms", "endq_ms"]),
    ("e2e", "pmap", ["insert_ms", "query_ms"]), ("e2e", "pmap5", ["insert_ms", "query_ms"]),
    ("alloc", "roomcreate_default", ["create_ms"]),
    ("alloc", "wide_default", ["coldbuild_ms", "build_ms", "clear_ms"]),
]
CATS = [("large", "large maps"), ("keys", "key patterns"), ("small", "small maps"),
        ("map", "map alone"), ("e2e", "end to end"), ("alloc", "allocator"), ("sweep", "size")]
NAMES = {
    "coordmap": "CoordMap",
    "akmap_inl_pack": "unordered_dense map, InnerGrids in the map",
    "akseg_pack": "unordered_dense segmented_map",
    "akmap_up_pmxA": "unordered_dense map, unique_ptr",
    "akmap_raw_pack": "unordered_dense map, raw pointer",
    "bflat192_inl_pack": "boost unordered_flat_map 1.92, InnerGrids in the map",
    "bnode192_pack": "boost unordered_node_map 1.92",
    "real_this": "unordered_dense map, in Bonxai (this branch)",
    "real_coordmap": "CoordMap, in Bonxai (d6b4d88)",
    "real_main": "std::unordered_map, in Bonxai (main)",
    "real_seg": "unordered_dense segmented_map, in Bonxai",
}
HARNESS = ["coordmap", "akmap_inl_pack", "akseg_pack", "akmap_up_pmxA", "akmap_raw_pack",
           "bflat192_inl_pack", "bnode192_pack"]
REAL = ["real_coordmap", "real_this", "real_seg", "real_main"]


def load(path):
    """(variant, workload) -> metric -> {round: value}"""
    runs = collections.defaultdict(lambda: collections.defaultdict(dict))
    if not os.path.exists(path):
        return runs
    for line in open(path):
        d = json.loads(line)
        if "failed" in d:
            continue
        for k, x in d.items():
            if isinstance(x, (int, float)) and k != "round":
                runs[(d["variant"], d["workload"])][k][d["round"]] = x
    return runs


def med(xs):
    return statistics.median(list(xs.values()))


def scores(runs, v, base, skip=()):
    per = collections.defaultdict(list)
    totals = []
    groups = [(c, w, ms) for c, w, ms in GROUPS if c not in skip]
    if (base, "sweep_rand") in runs and "sweep" not in skip:
        groups.append(("sweep", "sweep_rand", sorted(m for m in runs[(base, "sweep_rand")] if m.endswith("_ns"))))
    for c, w, ms in groups:
        at = bt = 0.0
        for m in ms:
            a, b = runs.get((v, w), {}).get(m), runs.get((base, w), {}).get(m)
            if not a or not b or med(a) <= 0 or med(b) <= 0:
                continue
            per[c].append(math.log(med(a) / med(b)))
            at += med(a)
            bt += med(b)
        if at and bt and c != "sweep":
            totals.append(math.log(at / bt))
    if not per:
        return None
    s = {c: math.exp(sum(x) / len(x)) for c, x in per.items()}
    s["cats"] = math.exp(sum(math.log(s[c]) for c in per) / len(per))
    pooled = sum(per.values(), [])
    s["all"] = math.exp(sum(pooled) / len(pooled))
    s["total"] = math.exp(sum(totals) / len(totals)) if totals else float("nan")
    return s


def table(runs, variants, base, skip=()):
    rows = [(scores(runs, v, base, skip), v) for v in variants]
    rows = [(s, v) for s, v in rows if s]
    cats = [c for c in CATS if any(c[0] in s for s, _ in rows)]
    print("| | " + " | ".join(l for _, l in cats) + " | **per category** | **total time** | every time |")
    print("|---|" + "---|" * (len(cats) + 3))
    for s, v in sorted(rows, key=lambda r: r[0]["cats"]):
        print(f"| {NAMES.get(v, v)} | " + " | ".join(f"{s[c]:.2f}" if c in s else "" for c, _ in cats) +
              f" | **{s['cats']:.2f}** | {s['total']:.2f} | {s['all']:.2f} |")
    return {v: s for s, v in rows}


def side_by_side(runs, variants):
    print("| workload | time | " + " | ".join(NAMES[v] for v in variants) + " |")
    print("|---|---|" + "---|" * len(variants))
    for c, w, ms in GROUPS:
        if c == "map":
            continue
        for m in ms:
            xs = [runs.get((v, w), {}).get(m) for v in variants]
            if not all(xs):
                continue
            b = med(xs[0])
            cells = [f"{med(x):.4g}" + ("" if i == 0 else f" ({med(x) / b:.2f})") for i, x in enumerate(xs)]
            print(f"| {w} | {m[:-3]} | " + " | ".join(cells) + " |")


def rule(h, p):
    print("\n## The rule, applied\n")
    if not all("e2e" in s for s in h.values()) or "real_this" in p and "e2e" not in p["real_this"]:
        print("End to end not measured for every variant: run everything (run_all.sh) first.")
        return
    passing, within = [], []
    for v, s in sorted(h.items(), key=lambda kv: kv[1]["all"]):
        if v == "coordmap":
            continue
        e2e = s.get("e2e", float("nan"))
        if s["all"] <= 0.95 and e2e <= 1.05:
            passing.append(v)
            verdict = "passes: score <= 0.95, end to end <= 1.05"
        elif s["all"] <= 0.95:
            verdict = "score <= 0.95, but end to end > 1.05"
        elif s["all"] < 1.05:
            within.append(v)
            verdict = "within 5%: prefer the established library"
        else:
            verdict = "CoordMap at least 5% better"
        print(f"- {NAMES[v]}: every time {s['all']:.3f}, end to end {e2e:.4f}: {verdict}")
    keep = not passing and not within
    print(f"\nRule 4, keep CoordMap only if at least 5% better than every other: {'yes' if keep else 'no'}")
    seg, flat = h.get("akseg_pack"), h.get("akmap_inl_pack")
    if seg and flat:
        print(f"unordered_dense map against segmented_map, every time: {flat['all'] / seg['all']:.3f} "
              f"(the map is preferred only at 0.95 or less)")
    for v in ("real_this", "real_seg"):
        t = p.get(v)
        if not t:
            continue
        print(f"\nBuilt into Bonxai, {NAMES[v]} against CoordMap: every time {t['all']:.3f}, "
              f"per category {t['cats']:.3f}, total time {t['total']:.3f}, end to end {t.get('e2e', float('nan')):.4f}")
        ok = t["all"] < 1.05 and t.get("e2e", 9) <= 1.05
        print(f"It {'confirms' if ok else 'does NOT confirm'} a switch to it (score < 1.05 and end to end <= 1.05).")


def growth(path):
    g = load(path)
    if not g:
        return
    print("\n## Growth to 970k roots, every insertion timed, medians\n")
    print("| | malloc | first build, ms | build, ms | worst insertion, first build, ms | worst insertion, ms |")
    print("|---|---|---|---|---|---|")
    for w in ("growth1000k", "growth1000k_default"):
        for v in HARNESS + REAL:
            d = g.get((v, w))
            if not d:
                continue
            m = lambda k: med(d[k]) if k in d else float("nan")
            print(f"| {NAMES[v]} | {'glibc defaults' if w.endswith('default') else 'memory kept'} | "
                  f"{m('cold_build_ms'):.0f} | {min(m('warm1_build_ms'), m('warm2_build_ms')):.0f} | "
                  f"{m('cold_worst_ms'):.1f} | {max(m('warm1_worst_ms'), m('warm2_worst_ms')):.1f} |")


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    runs = load(os.path.join(out, "final.jsonl"))
    print("# The harness, relative to CoordMap\n")
    h = table(runs, HARNESS, "coordmap")
    print("\nWithout the allocator category:\n")
    table(runs, HARNESS, "coordmap", skip=("alloc",))
    print("\n# Built into Bonxai, relative to CoordMap in Bonxai\n")
    p = table(runs, REAL, "real_coordmap")
    print("\nWithout the allocator category:\n")
    table(runs, REAL, "real_coordmap", skip=("alloc",))
    rule(h, p)
    print("\n# main, CoordMap and this branch, every time, medians in ms (ratio to main)\n")
    side_by_side(runs, [v for v in ["real_main", "real_coordmap", "real_this", "real_seg"] if any(k[0] == v for k in runs)])
    growth(os.path.join(out, "growth.jsonl"))


if __name__ == "__main__":
    main()
