#!/bin/bash
# setup.sh: fetches what the benchmark needs, outside the repository, into $ROOT_MAP_WORK
# (default: ../bonxai_root_map_work next to the repository):
#   ext/unordered_dense   ankerl::unordered_dense v5.1.0, for the harness's variants
#   ext/boost/*           boost.unordered 1.92 and the headers it needs
#   ext/nanovdb           NanoVDB's headers, OpenVDB v13.1.0, for nanovdb.py
#   main/                 a worktree of main as the branch started from (8d5904f)
#   coordmap/             a worktree of the branch when VoxelGrid used CoordMap (d6b4d88)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(git -C "$HERE" rev-parse --show-toplevel)
WORK=${ROOT_MAP_WORK:-$(dirname "$REPO")/bonxai_root_map_work}
mkdir -p "$WORK/ext/boost"
clone() {  # url tag dir
  [ -d "$3" ] || git -c advice.detachedHead=false clone -q --depth 1 --branch "$2" "$1" "$3"
}
clone https://github.com/martinus/unordered_dense v5.1.0 "$WORK/ext/unordered_dense"
for m in unordered assert config container_hash core describe mp11 predef static_assert \
         throw_exception type_traits; do
  clone "https://github.com/boostorg/$m" boost-1.92.0 "$WORK/ext/boost/$m"
done
[ -d "$WORK/ext/nanovdb" ] || {
  curl -sL https://github.com/AcademySoftwareFoundation/openvdb/archive/refs/tags/v13.1.0.tar.gz |
    tar xz -C "$WORK/ext" --wildcards 'openvdb-13.1.0/nanovdb/nanovdb/*'
  mkdir -p "$WORK/ext/nanovdb" && mv "$WORK/ext/openvdb-13.1.0/nanovdb/nanovdb" "$WORK/ext/nanovdb/"
  rm -rf "$WORK/ext/openvdb-13.1.0"
}
git -C "$REPO" fetch -q origin 2>/dev/null || true
[ -d "$WORK/main" ] || git -C "$REPO" worktree add -q --detach "$WORK/main" 8d5904f
[ -d "$WORK/coordmap" ] || git -C "$REPO" worktree add -q --detach "$WORK/coordmap" d6b4d88
echo "ready: $WORK"
