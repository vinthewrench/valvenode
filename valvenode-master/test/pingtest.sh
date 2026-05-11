#!/usr/bin/env bash
#
# ping_hammer.sh
#
# Quiet Valve Master ping-only RS-485 hammer test.
#
# Pattern:
#   power on
#   loop:
#     ping node 1
#     ping node 2
#     ping node 3
#     ping node 4
#   power off
#
# Usage:
#   chmod +x ping_hammer.sh
#   ./ping_hammer.sh
#   ./ping_hammer.sh 100
#
# Optional:
#   NODES="1 2 3 4" ./ping_hammer.sh 100
#   VERBOSE=1 ./ping_hammer.sh 100
#   VALVE_BIN=./valve ./ping_hammer.sh 100

set -u

VALVE_BIN="${VALVE_BIN:-./valve}"
LOOPS="${1:-100}"
VERBOSE="${VERBOSE:-0}"

# Override like:
#   NODES="2" ./ping_hammer.sh 100
#   NODES="1 2 3 4" ./ping_hammer.sh 100
NODES_TEXT="${NODES:-1 3 4 5 6}"
read -r -a NODES_ARR <<< "$NODES_TEXT"

POWER_SETTLE_SEC=1
BETWEEN_COMMAND_SEC=0.10
BETWEEN_LOOPS_SEC=0.25

pass_count=0
fail_count=0
start_time=$(date +%s)
fail_log=()

current_loop=0
current_phase="startup"

interrupted=0
cleaned_up=0

run_cmd()
{
    local label="$1"
    shift

    if [[ "$VERBOSE" != "0" ]]; then
        echo
        echo "==> $label"
        echo "+ $*"
        "$@"
    else
        "$@" >/tmp/ping_hammer_last.out 2>/tmp/ping_hammer_last.err
    fi

    local rc=$?

    if [[ "$rc" -eq 0 ]]; then
        pass_count=$((pass_count + 1))
        return 0
    fi

    fail_count=$((fail_count + 1))

    local err=""
    if [[ -s /tmp/ping_hammer_last.err ]]; then
        err="$(tr '\n' ' ' < /tmp/ping_hammer_last.err)"
    elif [[ -s /tmp/ping_hammer_last.out ]]; then
        err="$(tr '\n' ' ' < /tmp/ping_hammer_last.out)"
    fi

   fail_log+=("time='$(date "+%Y-%m-%d %H:%M:%S")' loop=${current_loop} phase='${current_phase}' label='${label}' rc=${rc} cmd='$*' output='${err}'")

    return "$rc"
}

print_summary()
{
    local end_time
    local elapsed
    local total

    end_time=$(date +%s)
    elapsed=$((end_time - start_time))
    total=$((pass_count + fail_count))

    echo
    echo "============================================================"
    echo "Ping hammer summary"
    echo "============================================================"
    echo "  loops requested:     $LOOPS"
    echo "  nodes tested:        ${NODES_ARR[*]}"
    echo "  total commands:      $total"
    echo "  passes:              $pass_count"
    echo "  fails:               $fail_count"
    echo "  elapsed seconds:     $elapsed"

    if [[ "$interrupted" -ne 0 ]]; then
        echo "  interrupted:         yes"
    else
        echo "  interrupted:         no"
    fi

    if [[ "$total" -gt 0 ]]; then
        awk -v p="$pass_count" -v f="$fail_count" -v t="$total" \
            'BEGIN {
                printf "  pass rate:           %.2f%%\n", (p / t) * 100;
                printf "  fail rate:           %.2f%%\n", (f / t) * 100;
            }'
    fi

    if [[ "$fail_count" -gt 0 ]]; then
        echo
        echo "Failures:"
        for item in "${fail_log[@]}"; do
            echo "  $item"
        done
    else
        echo
        echo "No failures recorded."
    fi
}

cleanup()
{
    if [[ "$cleaned_up" -ne 0 ]]; then
        return
    fi

    cleaned_up=1

    echo
    echo "Turning field power off..."
    "$VALVE_BIN" power off >/dev/null 2>&1 || true

    print_summary

    rm -f /tmp/ping_hammer_last.out /tmp/ping_hammer_last.err
}

handle_interrupt()
{
    interrupted=1
    echo
    echo "Interrupted. Quitting..."
    cleanup
    exit 130
}

trap handle_interrupt INT TERM
trap cleanup EXIT

if [[ ! -x "$VALVE_BIN" ]]; then
    echo "ERROR: $VALVE_BIN not found or not executable"
    exit 1
fi

echo "Valve bus ping hammer test"
echo "  valve binary: $VALVE_BIN"
echo "  loops:        $LOOPS"
echo "  nodes:        ${NODES_ARR[*]}"
echo "  verbose:      $VERBOSE"
echo

current_phase="power-on"
run_cmd "power on" "$VALVE_BIN" power on
sleep "$POWER_SETTLE_SEC"

for current_loop in $(seq 1 "$LOOPS"); do
    loop_fail_start=$fail_count
    loop_pass_start=$pass_count

    current_phase="ping"

    for node in "${NODES_ARR[@]}"; do
        run_cmd "ping node $node" "$VALVE_BIN" ping "$node"
        sleep "$BETWEEN_COMMAND_SEC"
    done

    loop_fails=$((fail_count - loop_fail_start))
    loop_passes=$((pass_count - loop_pass_start))

    if [[ "$loop_fails" -eq 0 ]]; then
        printf "loop %3d/%3d  PASS  pings=%2d  total_fail=%d\n" \
            "$current_loop" "$LOOPS" "$loop_passes" "$fail_count"
    else
        printf "loop %3d/%3d  FAIL  loop_fail=%2d  loop_pass=%2d  total_fail=%d\n" \
            "$current_loop" "$LOOPS" "$loop_fails" "$loop_passes" "$fail_count"
    fi

    sleep "$BETWEEN_LOOPS_SEC"
done

if [[ "$fail_count" -eq 0 ]]; then
    exit 0
fi

exit 1

