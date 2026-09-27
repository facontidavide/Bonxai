#!/usr/bin/env python3
"""run.py --out results.jsonl --rounds N --workloads w1,w2 [--bin DIR] [--core 3] variant...

Every (round, workload) runs each variant once, in its own process pinned to one core,
in a new random order. Resumes: what is already in --out is not run again."""
import json
import os
import random
import subprocess
import sys
import time

args = sys.argv[1:]
opts = {"--out": None, "--rounds": "3", "--workloads": None, "--bin": None, "--core": "3",
        "--timeout": "600"}
variants = []
i = 0
while i < len(args):
    if args[i] in opts:
        opts[args[i]] = args[i + 1]
        i += 2
    else:
        variants.append(args[i])
        i += 1
here = os.path.dirname(os.path.abspath(__file__))
bindir = opts["--bin"] or f"{here}/bin"
workloads = opts["--workloads"].split(",")
rounds = int(opts["--rounds"])
out = opts["--out"]

done = set()
if os.path.exists(out):
    for line in open(out):
        try:
            d = json.loads(line)
            done.add((d["round"], d["variant"], d["workload"]))
        except Exception:
            pass

total = rounds * len(workloads) * len(variants)
count = len(done)
t_start = time.time()
f = open(out, "a")
rng = random.Random(12345)
for r in range(rounds):
    for w in workloads:
        order = variants[:]
        rng.shuffle(order)
        for v in order:
            if (r, v, w) in done:
                continue
            try:
                p = subprocess.run(["taskset", "-c", opts["--core"], f"{bindir}/{v}", w],
                                   capture_output=True, text=True,
                                   timeout=float(opts["--timeout"]))
                line = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ""
                d = json.loads(line) if p.returncode == 0 and line else {
                    "variant": v, "workload": w, "failed": p.returncode,
                    "stderr": p.stderr[-300:]}
            except subprocess.TimeoutExpired:
                d = {"variant": v, "workload": w, "failed": "timeout"}
            d["round"] = r
            d["variant"] = v
            f.write(json.dumps(d) + "\n")
            f.flush()
            count += 1
    el = time.time() - t_start
    print(f"round {r + 1}/{rounds} done, {count}/{total}, {el / 60:.1f} min", file=sys.stderr,
          flush=True)
print("finished", file=sys.stderr)
