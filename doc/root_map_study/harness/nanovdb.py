#!/usr/bin/env python3
"""nanovdb.py [--cores "2 6"] [--rounds 5]: bonxai_core/benchmark/benchmark_nanovdb.cpp, built
against the headers of main, of CoordMap (d6b4d88) and of this branch, into
$ROOT_MAP_WORK/bin_nanovdb; then every benchmark in a process of its own (in one process,
the state the allocator was left in moves the creation timings by up to 2.5x), pinned, in a
new random order every round, the rounds shared by the cores. NanoVDB's own benchmarks run
from this branch's binary only. Results in $ROOT_MAP_WORK/results/nanovdb.jsonl, then the
medians, in ms, with the ratio to main. Run setup.sh first; resumes where it stopped."""
import collections
import json
import os
import random
import statistics
import subprocess
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = subprocess.run(["git", "-C", HERE, "rev-parse", "--show-toplevel"],
                      capture_output=True, text=True, check=True).stdout.strip()
WORK = os.environ.get("ROOT_MAP_WORK", os.path.join(os.path.dirname(REPO), "bonxai_root_map_work"))
BIN = os.path.join(WORK, "bin_nanovdb")
OUT = os.path.join(WORK, "results", "nanovdb.jsonl")
HEADERS = {"main": os.path.join(WORK, "main"), "coordmap": os.path.join(WORK, "coordmap"), "this": REPO}
opts = {"--cores": "2", "--rounds": "5"}
for i in range(1, len(sys.argv) - 1, 2):
    opts[sys.argv[i]] = sys.argv[i + 1]
cores, rounds = opts["--cores"].split(), int(opts["--rounds"])


def build():
    os.makedirs(BIN, exist_ok=True)
    src = os.path.join(REPO, "bonxai_core", "benchmark")
    jobs = [subprocess.Popen(["g++", "-std=c++17", "-O3", "-DNDEBUG", f"-I{d}/bonxai_core/include",
                              f"-I{src}", f"-I{WORK}/ext/nanovdb", f"{src}/benchmark_nanovdb.cpp",
                              "-o", f"{BIN}/{v}", "-lbenchmark", "-lpthread"])
            for v, d in HEADERS.items() if not os.path.exists(f"{BIN}/{v}")]
    if any(j.wait() for j in jobs):
        sys.exit("build failed")


def tests():
    names = subprocess.run([f"{BIN}/this", "--benchmark_list_tests"], capture_output=True,
                           text=True, check=True).stdout.split("\n")
    return [(v, n) for n in names if n for v in (["this"] if n.startswith("NanoVDB") else HEADERS)]


def run(core, my_rounds, todo, done, lock):
    for r in my_rounds:
        order = todo[:]
        random.Random(777 + r).shuffle(order)  # the order of a round does not depend on who runs it
        for v, n in order:
            if (r, v, n) in done:
                continue
            p = subprocess.run(["taskset", "-c", core, f"{BIN}/{v}", "--benchmark_format=json",
                                f"--benchmark_filter=^{n.replace('.', chr(92) + '.')}$"],
                               capture_output=True, text=True)
            d = {"round": r, "variant": v, "test": n}
            try:
                b = json.loads(p.stdout)["benchmarks"][0]
                d["ms"] = b["real_time"] * {"ns": 1e-6, "us": 1e-3, "ms": 1, "s": 1e3}[b["time_unit"]]
                d.update({k: b[k] for k in ("Bonxai_MB", "NanoVDB_ReadOnly_MB", "cells") if k in b})
            except (ValueError, KeyError, IndexError):
                d["failed"] = p.stderr[-300:]
            with lock, open(OUT, "a") as f:
                f.write(json.dumps(d) + "\n")
        print(f"core {core}: round {r + 1}/{rounds} done", file=sys.stderr, flush=True)


def report():
    res = collections.defaultdict(list)
    for line in open(OUT):
        d = json.loads(line)
        if "ms" in d:
            key = "Bonxai_MB" if "Bonxai_MB" in d else "ms"
            res[(d["test"].replace("/min_time:1.000", "").replace("Bonxai_NV_", ""), d["variant"])].append(d[key])
    print("| benchmark | main | CoordMap | this | spread (max/min, this) |\n|---|---|---|---|---|")
    for t in dict.fromkeys(k[0] for k in res):
        m = {v: statistics.median(res[(t, v)]) for v in HEADERS if (t, v) in res}
        if "main" in m:
            cells = [f"{m['main']:.4g}"] + [f"{m[v]:.4g} ({m[v] / m['main']:.2f})" if v in m else ""
                                            for v in ("coordmap", "this")]
        else:
            cells = ["", "", f"{m['this']:.4g}"]
        xs = res[(t, "this")]
        print(f"| {t} | " + " | ".join(cells) + f" | {max(xs) / min(xs):.2f} |")


if __name__ == "__main__":
    build()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    done = set()
    if os.path.exists(OUT):
        done = {(d["round"], d["variant"], d["test"]) for d in map(json.loads, open(OUT))}
    todo, lock = tests(), threading.Lock()
    threads = [threading.Thread(target=run, args=(c, range(i, rounds, len(cores)), todo, done, lock))
               for i, c in enumerate(cores)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    report()
