#!/bin/bash
# run_all.sh: every variant on every workload, in rounds, each run a process of its own pinned
# to one core, in a new random order every round; then the growth runs; then analyze.py.
#   ROOT_MAP_CORES   the cores to run on (default 2), one runner each, sharing the rounds:
#                    pick physical cores with nothing else on them, hyperthread siblings idle
#   ROOT_MAP_ROUNDS  rounds (default 5)
# Resumes where it stopped: runs already in the results are not run again.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
WORK=${ROOT_MAP_WORK:-$(dirname "$REPO")/bonxai_root_map_work}
read -ra CORES <<< "${ROOT_MAP_CORES:-2}"
ROUNDS=${ROOT_MAP_ROUNDS:-5}
OUT=$WORK/results
mkdir -p "$OUT"
W=wide,wide1m,widepos,vg_rand_250000,vg_dense_250000,vg_plane_250000,vg_line_250000,vg_stride_250000,vg_rand_1000,vg_rand_10000,room,map_rand_1000,map_rand_100000,map_rand_1000000,map_dense_100000,rays10,pmap,pmap5,roomcreate_default,wide_default,sweep_rand
V="coordmap akmap_inl_pack akseg_pack akmap_up_pmxA akmap_raw_pack bflat192_inl_pack bnode192_pack real_this real_coordmap real_main"
cd "$HERE"
run() {  # name workloads: one runner per core, then their results merged into $OUT/name.jsonl
  for i in "${!CORES[@]}"; do
    python3 run.py --bin "$WORK/bin" --out "$OUT/$1.core${CORES[i]}.jsonl" --rounds "$ROUNDS" \
      --first "$i" --stride "${#CORES[@]}" --workloads "$2" --core "${CORES[i]}" --timeout 900 $V &
  done
  wait
  cat "$OUT/$1".core*.jsonl > "$OUT/$1.jsonl"
}
run final "$W" 2>&1 | tee "$OUT/final.log"
run growth growth1000k,growth1000k_default 2>&1 | tee "$OUT/growth.log"
python3 analyze.py "$OUT" | tee "$OUT/analysis.md"
