#!/usr/bin/env python3
"""build.py [--no-pmap] [variant ...]: one binary per variant, into $ROOT_MAP_WORK/bin.

The harness variants compile a copy of VoxelGrid (include/bonxai/bonxai.hpp) whose root
map is a template parameter, with each map behind one interface (maps.hpp). The real_*
variants compile the headers of a checkout as they are:
  real_this      this branch: CoordMap, as real_coordmap, with the fixes that came after
  real_coordmap  the branch at d6b4d88, when VoxelGrid used CoordMap
  real_main      main at 8d5904f: std::unordered_map
Run setup.sh first."""
import concurrent.futures as cf
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = subprocess.run(["git", "-C", HERE, "rev-parse", "--show-toplevel"],
                      capture_output=True, text=True, check=True).stdout.strip()
WORK = os.environ.get("ROOT_MAP_WORK", os.path.join(os.path.dirname(REPO), "bonxai_root_map_work"))
EXT = os.path.join(WORK, "ext")
BOOST = ["unordered", "assert", "config", "container_hash", "core", "describe", "mp11", "predef",
         "static_assert", "throw_exception", "type_traits"]
LIBS = {
    "ANKERL": [f"-I{EXT}/unordered_dense/include", "-DWITH_ANKERL"],
    "BOOST": [f"-I{EXT}/boost/{m}/include" for m in BOOST] + ["-DWITH_BOOST"],
}
# name: (policy, library): the contenders of the last round, see ../README.md
HARNESS = {
    "coordmap": ("mx::CoordMapPolicy", None),
    "akmap_inl_pack": ("mx::Inline<mx::AnkerlMap, mx::HPack>", "ANKERL"),
    "akmap_up_pmxA": ("mx::Ptr<mx::AnkerlMap, mx::HPmxA>", "ANKERL"),
    "akmap_raw_pack": ("mx::Raw<mx::AnkerlMap, mx::HPack>", "ANKERL"),
    "akseg_pack": ("mx::SegNode<mx::AnkerlSeg, mx::HPack>", "ANKERL"),
    "bflat192_inl_pack": ("mx::Inline<mx::BoostFlat, mx::HPack>", "BOOST"),
    "bnode192_pack": ("mx::Node<mx::BoostNode, mx::HPack>", "BOOST"),
}
REAL = {
    "real_this": os.path.join(REPO, "bonxai_core", "include"),
    "real_coordmap": os.path.join(WORK, "coordmap", "bonxai_core", "include"),
    "real_main": os.path.join(WORK, "main", "bonxai_core", "include"),
}


def eigen():
    r = subprocess.run(["pkg-config", "--cflags", "eigen3"], capture_output=True, text=True)
    return r.stdout.split() if r.returncode == 0 else ["-I/usr/include/eigen3"]


def command(name, pmap, out):
    cmd = [os.environ.get("CXX", "g++"), "-std=c++17", "-O3", "-DNDEBUG", f'-DVARIANT_NAME="{name}"',
           "-Wno-deprecated-declarations"]
    if name in REAL:
        cmd += [f"-I{REAL[name]}", "-DREAL"]
    else:
        policy, lib = HARNESS[name]
        cmd += [f"-I{HERE}/include", f"-I{HERE}", f"-DBONXAI_DEFAULT_POLICY={policy}"]
        cmd += LIBS[lib] if lib else []
    cmd += [f"-I{REPO}/bonxai_core/benchmark"]
    if pmap:
        cmd += ["-DWITH_PMAP", f"-I{REPO}/bonxai_map/include", f"-I{REPO}/bonxai_map/src"] + eigen()
    return cmd + [os.path.join(HERE, "bench.cpp"), "-o", os.path.join(out, name)]


def main():
    args = sys.argv[1:]
    pmap = "--no-pmap" not in args
    names = [a for a in args if not a.startswith("--")] or list(HARNESS) + list(REAL)
    out = os.path.join(WORK, "bin")
    os.makedirs(out, exist_ok=True)
    with cf.ThreadPoolExecutor(os.cpu_count()) as pool:
        jobs = {n: pool.submit(subprocess.run, command(n, pmap, out), capture_output=True, text=True)
                for n in names}
        failed = 0
        for n, j in jobs.items():
            r = j.result()
            print("ok" if r.returncode == 0 else "FAILED", n)
            if r.returncode:
                failed += 1
                print(r.stderr[-3000:])
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
