#!/usr/bin/env bash
#
# hammer.sh
#
# Quiet Valve Master RS-485 hammer test.
#
# Pattern:
#   power on
#   ping nodes
#   initial status nodes
#   loop:
#     set ON  nodes
#     status  nodes
#     set OFF nodes
#     status  nodes
#   final status nodes
#   power off
#
# Usage:
#   chmod +x hammer.sh
#   ./hammer.sh
#   ./hammer.sh 25
#
# Optional:
#   CHANNEL=2 ./hammer.sh 25
#   VERBOSE=1 ./hammer.sh 25
#   VALVE_BIN=./valve ./hammer.sh 25
#   NODES="1 3 4 5 6" ./hammer.sh 25
#   RETRY_ON_FAIL=1 ./hammer.sh 25

set -u

VALVE_BIN="${VALVE_BIN:-./valve}"
LOOPS="${1:-25}"
CHANNEL="${CHANNEL:-1}"
VERBOSE="${VERBOSE:-0}"
RETRY_ON_FAIL="${RETRY_ON_FAIL:-0}"

if [[ -n "${NODES:-}" ]]; then
    read -r -a NODE_LIST <<< "$NODES"
else
    NODE_LIST=(1 3 4 5 6)
fi

POWER_SETTLE_SEC="${POWER_SETTLE_SEC:-1}"
BETWEEN_COMMAND_SEC="${BETWEEN_COMMAND_SEC:-0.10}"
BETWEEN_PHASE_SEC="${BETWEEN_PHASE_SEC:-0.25}"
BETWEEN_LOOPS_SEC="${BETWEEN_LOOPS_SEC:-0.50}"

TMP_OUT="/tmp/hammer_last.out"
TMP_ERR="/tmp/hammer_last.err"

pass_count=0
fail_count=0
retry_pass_count=0
retry_fail_count=0

start_time=$(date +%s)
fail_log=()

current_loop=0
current_phase="startup"

interrupted=0
cleaned_up=0

timestamp()
{
    date "+%Y-%m-%d %H:%M:%S"
}

run_once()
{
    local label="$1"
    shift

    : >"$TMP_OUT"
    : >"$TMP_ERR"

    if [[ "$VERBOSE" != "0" ]]; then
        echo
        echo "==> $label"
        echo "+ $*"
        "$@"
    else
        "$@" >"$TMP_OUT" 2>"$TMP_ERR"
    fi

    return $?
}

capture_output()
{
    local err=""

    if [[ -s "$TMP_ERR" ]]; then
        err="$(tr '\n' ' ' < "$TMP_ERR")"
    elif [[ -s "$TMP_OUT" ]]; then
        err="$(tr '\n' ' ' < "$TMP_OUT")"
    fi

    printf "%s" "$err"
}

run_cmd()
{
    local label="$1"
    shift

    run_once "$label" "$@"
    local rc=$?

    if [[ "$rc" -eq 0 ]]; then
        pass_count=$((pass_count + 1))
        return 0
    fi

    fail_count=$((fail_count + 1))

    local err
    err="$(capture_output)"

    fail_log+=("time='$(timestamp)' loop=${current_loop} phase='${current_phase}' label='${label}' rc=${rc} cmd='$*' output='${err}'")

    if [[ "$RETRY_ON_FAIL" != "0" ]]; then
        sleep "$BETWEEN_COMMAND_SEC"

        run_once "retry: $label" "$@"
        local retry_rc=$?

        if [[ "$retry_rc" -eq 0 ]]; then
            retry_pass_count=$((retry_pass_count + 1))
            fail_log+=("time='$(timestamp)' loop=${current_loop} phase='${current_phase}' label='${label}' retry='PASS'")
        else
            retry_fail_count=$((retry_fail_count + 1))
            local retry_err
            retry_err="$(capture_output)"
            fail_log+=("time='$(timestamp)' loop=${current_loop} phase='${current_phase}' label='${label}' retry='FAIL' retry_rc=${retry_rc} retry_output='${retry_err}'")
        fi
    fi

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
    echo "Hammer summary"
    echo "============================================================"
    echo "  loops requested:     $LOOPS"
    echo "  nodes tested:        ${NODE_LIST[*]}"
    echo "  channel tested:      $CHANNEL"
    echo "  total commands:      $total"
    echo "  passes:              $pass_count"
    echo "  first-try fails:     $fail_count"
    echo "  retry passes:        $retry_pass_count"
    echo "  retry fails:         $retry_fail_count"
    echo "  elapsed seconds:     $elapsed"

    if [[ "$interrupted" -ne 0 ]]; then
        echo "  interrupted:         yes"
    else
        echo "  interrupted:         no"
    fi

    if [[ "$total" -gt 0 ]]; then
        awk -v p="$pass_count" -v f="$fail_count" -v t="$total" \
            'BEGIN {
                printf "  first-try pass rate: %.2f%%\n", (p / t) * 100;
                printf "  first-try fail rate: %.2f%%\n", (f / t) * 100;
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

    rm -f "$TMP_OUT" "$TMP_ERR"
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
    echo "ERROR: $VALVE_BIN not found or not executable: $VALVE_BIN"
    exit 1
fi

if ! [[ "$LOOPS" =~ ^[0-9]+$ ]] || [[ "$LOOPS" -lt 1 ]]; then
    echo "ERROR: loop count must be >= 1"
    exit 1
fi

if ! [[ "$CHANNEL" =~ ^[0-9]+$ ]] || [[ "$CHANNEL" -lt 1 ]] || [[ "$CHANNEL" -gt 16 ]]; then
    echo "ERROR: channel must be 1..16"
    exit 1
fi

echo "Valve bus hammer test"
echo "  valve binary:  $VALVE_BIN"
echo "  loops:         $LOOPS"
echo "  nodes:         ${NODE_LIST[*]}"
echo "  channel:       $CHANNEL"
echo "  verbose:       $VERBOSE"
echo "  retry on fail: $RETRY_ON_FAIL"
echo

current_phase="power-on"
run_cmd "power on" "$VALVE_BIN" power on
sleep "$POWER_SETTLE_SEC"

current_phase="initial-ping"
for node in "${NODE_LIST[@]}"; do
    run_cmd "ping node $node" "$VALVE_BIN" ping "$node"
    sleep "$BETWEEN_COMMAND_SEC"
done

current_phase="initial-status"
for node in "${NODE_LIST[@]}"; do
    run_cmd "initial status node $node channel $CHANNEL" \
        "$VALVE_BIN" channel "$node" "$CHANNEL" status
    sleep "$BETWEEN_COMMAND_SEC"
done

for current_loop in $(seq 1 "$LOOPS"); do
    loop_fail_start=$fail_count
    loop_pass_start=$pass_count

    current_phase="set-on"
    for node in "${NODE_LIST[@]}"; do
        run_cmd "set node $node channel $CHANNEL ON" \
            "$VALVE_BIN" set "$node" "$CHANNEL" on
        sleep "$BETWEEN_COMMAND_SEC"
    done

    sleep "$BETWEEN_PHASE_SEC"

    current_phase="status-after-on"
    for node in "${NODE_LIST[@]}"; do
        run_cmd "status node $node channel $CHANNEL after ON" \
            "$VALVE_BIN" channel "$node" "$CHANNEL" status
        sleep "$BETWEEN_COMMAND_SEC"
    done

    sleep "$BETWEEN_PHASE_SEC"

    current_phase="set-off"
    for node in "${NODE_LIST[@]}"; do
        run_cmd "set node $node channel $CHANNEL OFF" \
            "$VALVE_BIN" set "$node" "$CHANNEL" off
        sleep "$BETWEEN_COMMAND_SEC"
    done

    sleep "$BETWEEN_PHASE_SEC"

    current_phase="status-after-off"
    for node in "${NODE_LIST[@]}"; do
        run_cmd "status node $node channel $CHANNEL after OFF" \
            "$VALVE_BIN" channel "$node" "$CHANNEL" status
        sleep "$BETWEEN_COMMAND_SEC"
    done

    loop_fails=$((fail_count - loop_fail_start))
    loop_passes=$((pass_count - loop_pass_start))

    if [[ "$loop_fails" -eq 0 ]]; then
        printf "%s  loop %3d/%3d  PASS  cmds=%2d  total_fail=%d\n" \
            "$(timestamp)" "$current_loop" "$LOOPS" "$loop_passes" "$fail_count"
    else
        printf "%s  loop %3d/%3d  FAIL  loop_fail=%2d  loop_pass=%2d  total_fail=%d\n" \
            "$(timestamp)" "$current_loop" "$LOOPS" "$loop_fails" "$loop_passes" "$fail_count"
    fi

    sleep "$BETWEEN_LOOPS_SEC"
done

current_phase="final-status"
for node in "${NODE_LIST[@]}"; do
    run_cmd "final status node $node channel $CHANNEL" \
        "$VALVE_BIN" channel "$node" "$CHANNEL" status
    sleep "$BETWEEN_COMMAND_SEC"
done

if [[ "$fail_count" -eq 0 ]]; then
    exit 0
fi

exit 1

