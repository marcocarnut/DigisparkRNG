#!/bin/sh
# Fetch and build NIST's SP 800-90B entropy assessment tool (ea_non_iid) with
# the libraries it needs that aren't installed system-wide (libdivsufsort,
# jsoncpp), all under third_party/. Result: tools/ea_non_iid.
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
TP=$ROOT/third_party
PREFIX=$TP/prefix
JOBS=2  # keep the laptop cool
mkdir -p "$TP"
cd "$TP"

fetch() {  # url dir
  [ -d "$2" ] || git clone --depth 1 "$1" "$2"
}

fetch https://github.com/y-256/libdivsufsort.git libdivsufsort
cmake -S libdivsufsort -B libdivsufsort/build -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBUILD_DIVSUFSORT64=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build libdivsufsort/build -j$JOBS
cmake --install libdivsufsort/build

fetch https://github.com/open-source-parsers/jsoncpp.git jsoncpp
cmake -S jsoncpp -B jsoncpp/build -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBUILD_SHARED_LIBS=OFF -DBUILD_STATIC_LIBS=ON -DJSONCPP_WITH_TESTS=OFF \
  -DJSONCPP_WITH_POST_BUILD_UNITTEST=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build jsoncpp/build -j$JOBS
cmake --install jsoncpp/build

fetch https://github.com/usnistgov/SP800-90B_EntropyAssessment.git SP800-90B_EntropyAssessment
cd SP800-90B_EntropyAssessment/cpp
make non_iid \
  CXXFLAGS="-std=c++11 -fopenmp -O2 -ffloat-store -march=native -I$PREFIX/include -I$PREFIX/include/json" \
  LIB="-L$PREFIX/lib -lbz2 -lpthread -ldivsufsort -ldivsufsort64" \
  SHARED_LIB="$PREFIX/lib/libjsoncpp.a -lcrypto"
cp ea_non_iid "$ROOT/tools/"
echo "built $ROOT/tools/ea_non_iid"
