#!/usr/bin/env bash
#
# Host test runner. Compiles and runs every test/test_*.cpp against the
# firmware headers, using the Arduino stub in place of the real core.
#
# There is no robot available to test on, so these host tests are the only
# executable check that the firmware behaves as intended.

set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CXX="${CXX:-g++}"
CXXFLAGS="-std=c++17 -Wall -Wextra -I ${ROOT}/test -I ${ROOT}/line_following_robot"

BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "${BUILD_DIR}"' EXIT

failed=0
ran=0

for src in "${ROOT}"/test/test_*.cpp; do
  [ -e "${src}" ] || continue
  name="$(basename "${src}" .cpp)"
  bin="${BUILD_DIR}/${name}.exe"

  echo "== ${name}"

  if ! ${CXX} ${CXXFLAGS} "${src}" -o "${bin}"; then
    echo "   COMPILE FAILED: ${src}"
    failed=$((failed + 1))
    continue
  fi

  ran=$((ran + 1))
  if ! "${bin}"; then
    failed=$((failed + 1))
  fi
done

if [ "${ran}" -eq 0 ]; then
  echo "no test files found"
  exit 1
fi

if [ "${failed}" -ne 0 ]; then
  echo "FAILED: ${failed} test binary/binaries reported failures"
  exit 1
fi

echo "OK: ${ran} test binaries passed"
exit 0
