#!/bin/bash
# Vendors ankerl::unordered_dense (https://github.com/martinus/unordered_dense, MIT) as
# Bonxai::unordered_dense, with its macros renamed, so that it cannot clash with another
# copy of the library in the same program.
#
# usage: update_unordered_dense.sh <checkout of unordered_dense at the wanted tag>
set -euo pipefail
SRC=$1/include/ankerl
DST=$(dirname "$0")
for f in unordered_dense.h stl.h; do
  sed -e 's/ANKERL_UNORDERED_DENSE_/BONXAI_UD_/g' \
      -e 's/ANKERL_MEMORY_RESOURCE_IS_BAD/BONXAI_UD_MEMORY_RESOURCE_IS_BAD/g' \
      -e 's/ANKERL_STL_H/BONXAI_UD_STL_H/g' \
      -e 's/namespace ankerl::unordered_dense/namespace Bonxai::unordered_dense/g' \
      -e 's/ankerl::unordered_dense::/Bonxai::unordered_dense::/g' \
      "$SRC/$f" > "$DST/$f"
done
