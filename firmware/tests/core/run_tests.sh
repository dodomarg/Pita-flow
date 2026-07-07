#!/usr/bin/env bash
# Builds and runs the host-side unit tests for the hardware-independent
# `core` firmware component. Requires only a plain C toolchain (gcc/clang),
# not the ESP-IDF SDK.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
core_dir="${script_dir}/../../components/core"

cc="${CC:-gcc}"

"${cc}" -std=c11 -Wall -Wextra \
    -I "${core_dir}/include" \
    "${script_dir}/run_tests.c" \
    "${core_dir}"/src/*.c \
    -lm \
    -o /tmp/pita_flow_core_tests

/tmp/pita_flow_core_tests
