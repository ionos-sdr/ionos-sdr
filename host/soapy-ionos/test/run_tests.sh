#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# run_tests.sh - SoapyIonos tests against the pty mock (Linux)
#   Copyright (c) 2026 Zoltan Doczi HA7DCD
#
#   test/run_tests.sh <build-dir>          (after cmake --build <build-dir>)
#
# Uses the freshly built module via SOAPY_SDR_PLUGIN_PATH, so nothing has to be
# installed. Needs python3 + numpy and SoapySDRUtil.

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$(cd "${1:-$HERE/../build}" && pwd)"
WORK="$(mktemp -d)"
PORT="$WORK/ionos0"
export SOAPY_SDR_PLUGIN_PATH="$BUILD"
SECS="${SECS:-12}"
fails=0
MOCK_PID=""

start_mock() {
    python3 "$HERE/ionos_mock.py" --quiet --link "$PORT" --stats "$WORK/mock.json" "$@" > "$WORK/mock.log" 2>&1 &
    MOCK_PID=$!
    for _ in $(seq 50); do [ -e "$PORT" ] && break; sleep 0.1; done
}
stop_mock() {
    [ -n "$MOCK_PID" ] && kill "$MOCK_PID" 2>/dev/null && wait "$MOCK_PID" 2>/dev/null
    MOCK_PID=""
    [ -f "$WORK/mock.json" ] && echo "    mock: $(cat "$WORK/mock.json")"
}
result() {
    if [ "$1" -eq 0 ]; then echo "PASS  $2"; else echo "FAIL  $2"; fails=$((fails + 1)); fi
}
trap 'stop_mock; rm -rf "$WORK"' EXIT

echo "== parser unit test"
"$BUILD/ionos_parser_test" > "$WORK/parser.log"; result $? "parser: 50 random streams with junk, drops, 128-byte cuts, SPC1/MSG1"

echo "== discovery"
start_mock
IONOS_PORTS="$PORT" SoapySDRUtil --find="driver=ionos" > "$WORK/find.log" 2>&1
grep -q "board = BRD4265B" "$WORK/find.log"; result $? "SoapySDRUtil --find=driver=ionos (IONOS_PORTS)"
SoapySDRUtil --probe="driver=ionos,port=$PORT" > "$WORK/probe.log" 2>&1
grep -q "Sample rates: 0.00625, 0.00833333, 0.0125, 0.0166667, 0.025, 0.05, 0.1, 0.2 MSps" "$WORK/probe.log" &&
    grep -q "RF freq range: \[430, 440.025\] MHz" "$WORK/probe.log"
result $? "SoapySDRUtil --probe: rates and range from CAP?"
stop_mock

run_stream() {  # name, loss args (mock and test), mock-only args, test-only args
    local name="$1" loss="$2" mextra="$3" targs="$4"
    echo "== stream: $name"
    # shellcheck disable=SC2086
    start_mock $loss $mextra
    # shellcheck disable=SC2086
    "$BUILD/ionos_stream_test" --args "driver=ionos,port=$PORT" --secs "$SECS" $loss $targs > "$WORK/stream.log" 2>&1
    local rc=$?
    grep -E "^read |^lost_blocks|^zero-filled|^tone:|^\[FAIL\]" "$WORK/stream.log" | sed 's/^/    /'
    stop_mock
    result $rc "$name"
}

LOSS="--drop-every 500 --drop-burst 3 --cut-every 777"
run_stream "200 ksps CF32 clean"                         ""      ""                               "--rate 200000 --format CF32"
run_stream "200 ksps CF32 drops+cuts, SPC1+MSG1 mixed in" "$LOSS" "--spc-every 1000 --err-every 3" "--rate 200000 --format CF32"
run_stream "200 ksps CS16 drops+cuts"                    "$LOSS" ""                               "--rate 200000 --format CS16"
run_stream "100 ksps CS8 drops+cuts"                     "$LOSS" ""                               "--rate 100000 --format CS8"
run_stream "50 ksps CF32 bursts of 20 blocks"            "--drop-every 300 --drop-burst 20" ""    "--rate 50000 --format CF32"

echo
if [ $fails -eq 0 ]; then echo "ALL TESTS PASSED"; else echo "$fails TEST(S) FAILED"; fi
exit $fails
