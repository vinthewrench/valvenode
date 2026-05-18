#!/usr/bin/env bash
#
# hammer.sh
#
# Quiet Valve Master RS-485 hammer test.
#
# Pattern:
#   power on
#   ping nodes
#   initial status nodes/channels
#   loop:
#     set ON  nodes/channels
#     status  nodes/channels
#     set OFF nodes/channels
#     status  nodes/channels
#   final status nodes/channels
#   power off
#
# Usage:
#   chmod +x hammer.sh
#   ./hammer.sh
#   ./hammer.sh 25
#
# Defaults:
#   NODES="1 2 3 4 5 6"
#   CHANNEL="1 2"
#
# Optional:
#   CHANNEL=1 ./hammer.sh 25
#   CHANNEL="1 2" ./hammer.sh 25
#   VERBOSE=1 ./hammer.sh 25
#   VALVE_BIN=/path/to/valve ./hammer.sh 25
#   NODES="1 2 3 4 5" ./hammer.sh 25
#   RETRY_ON_FAIL=1 ./hammer.sh 25
#   CMD_TIMEOUT_SEC=5 ./hammer.sh 25
#   FAIL_LOG_FILE=hammer_failures.log ./hammer.sh 25
#
# Valve binary search priority:
#   1. VALVE_BIN override
#   2. ./valve in current directory
#   3. valve from PATH
#
# Capture full run:
#   ./hammer.sh 1000 2>&1 | tee hammer-$(date +%Y%m%d-%H%M%S).log

set -u

if [[ -n "${VALVE_BIN:-}" ]]; then
    VALVE_BIN="$VALVE_BIN"
elif [[ -x "./valve" ]]; then
    VALVE_BIN="./valve"
else
    VALVE_BIN="$(command -v valve || true)"
fi

LOOPS="${1:-25}"
CHANNEL="${CHANNEL:-1 2}"
VERBOSE="${VERBOSE:-0}"
RETRY_ON_FAIL="${RETRY_ON_FAIL:-0}"
CMD_TIMEOUT_SEC="${CMD_TIMEOUT_SEC:-5}"
FAIL_LOG_FILE="${FAIL_LOG_FILE:-hammer_failures.log}"

if [[ -n "${NODES:-}" ]]; then
    read -r -a NODE_LIST <<< "$NODES"
else
    NODE_LIST=(1 2 3 4 5 6)
fi

read -r -a CHANNEL_LIST <<< "$CHANNEL"

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
timeout_count=0

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

log_failure_line()
{
    local line="$1"

    fail_log+=("$line")
    echo "$line" >> "$FAIL_LOG_FILE"
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
        echo "+ timeout $CMD_TIMEOUT_SEC $*"
        timeout "$CMD_TIMEOUT_SEC" "$@"
    else
        timeout "$CMD_TIMEOUT_SEC" "$@" >"$TMP_OUT" 2>"$TMP_ERR"
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

    if [[ "$rc" -eq 124 ]]; then
        timeout_count=$((timeout_count + 1))
    fi

    local err
    err="$(capture_output)"

    local line
    line="time='$(timestamp)' loop=${current_loop} phase='${current_phase}' label='${label}' rc=${rc} cmd='$*' output='${err}'"
    log_failure_line "$line"

    if [[ "$RETRY_ON_FAIL" != "0" ]]; then
        sleep "$BETWEEN_COMMAND_SEC"

        run_once "retry: $label" "$@"
        local retry_rc=$?

        if [[ "$retry_rc" -eq 0 ]]; then
            retry_pass_count=$((retry_pass_count + 1))

            line="time='$(timestamp)' loop=${current_loop} phase='${current_phase}' label='${label}' retry='PASS'"
            log_failure_line "$line"
        else
            retry_fail_count=$((retry_fail_count + 1))

            if [[ "$retry_rc" -eq 124 ]]; then
                timeout_count=$((timeout_count + 1))
            fi

            local retry_err
            retry_err="$(capture_output)"

            line="time='$(timestamp)' loop=${current_loop} phase='${current_phase}' label='${label}' retry='FAIL' retry_rc=${retry_rc} retry_output='${retry_err}'"
            log_failure_line "$line"
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
    echo "  channels tested:     ${CHANNEL_LIST[*]}"
    echo "  total commands:      $total"
    echo "  passes:              $pass_count"
    echo "  first-try fails:     $fail_count"
    echo "  retry passes:        $retry_pass_count"
    echo "  retry fails:         $retry_fail_count"
    echo "  command timeouts:    $timeout_count"
    echo "  command timeout sec: $CMD_TIMEOUT_SEC"
    echo "  failure log file:    $FAIL_LOG_FILE"
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
    timeout "$CMD_TIMEOUT_SEC" "$VALVE_BIN" power off >/dev/null 2>&1 || true

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

if [[ -z "$VALVE_BIN" ]] || [[ ! -x "$VALVE_BIN" ]]; then
    echo "ERROR: valve not found or not executable"
    echo "Set VALVE_BIN=/path/to/valve, put ./valve in this directory, or make sure valve is in your PATH."
    exit 1
fi

if ! command -v timeout >/dev/null 2>&1; then
    echo "ERROR: timeout command not found"
    echo "On macOS, install coreutils and use gtimeout, or run this on Linux."
    exit 1
fi

if ! [[ "$LOOPS" =~ ^[0-9]+$ ]] || [[ "$LOOPS" -lt 1 ]]; then
    echo "ERROR: loop count must be >= 1"
    exit 1
fi

if [[ "${#NODE_LIST[@]}" -lt 1 ]]; then
    echo "ERROR: at least one node must be specified"
    exit 1
fi

for node in "${NODE_LIST[@]}"; do
    if ! [[ "$node" =~ ^[0-9]+$ ]] || [[ "$node" -lt 1 ]] || [[ "$node" -gt 254 ]]; then
        echo "ERROR: node must be 1..254: $node"
        exit 1
    fi
done

if [[ "${#CHANNEL_LIST[@]}" -lt 1 ]]; then
    echo "ERROR: at least one channel must be specified"
    exit 1
fi

for channel in "${CHANNEL_LIST[@]}"; do
    if ! [[ "$channel" =~ ^[0-9]+$ ]] || [[ "$channel" -lt 1 ]] || [[ "$channel" -gt 16 ]]; then
        echo "ERROR: channel must be 1..16: $channel"
        exit 1
    fi
done

: > "$FAIL_LOG_FILE"

echo "Valve bus hammer test"
echo "  valve binary:        $VALVE_BIN"
echo "  loops:               $LOOPS"
echo "  nodes:               ${NODE_LIST[*]}"
echo "  channels:            ${CHANNEL_LIST[*]}"
echo "  verbose:             $VERBOSE"
echo "  retry on fail:       $RETRY_ON_FAIL"
echo "  command timeout sec: $CMD_TIMEOUT_SEC"
echo "  failure log file:    $FAIL_LOG_FILE"
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
for channel in "${CHANNEL_LIST[@]}"; do
    for node in "${NODE_LIST[@]}"; do
        run_cmd "initial status node $node channel $channel" \
            "$VALVE_BIN" channel "$node" "$channel" status
        sleep "$BETWEEN_COMMAND_SEC"
    done
done

for current_loop in $(seq 1 "$LOOPS"); do
    loop_fail_start=$fail_count
    loop_pass_start=$pass_count

    current_phase="set-on"
    for channel in "${CHANNEL_LIST[@]}"; do
        for node in "${NODE_LIST[@]}"; do
            run_cmd "set node $node channel $channel ON" \
                "$VALVE_BIN" set "$node" "$channel" on
            sleep "$BETWEEN_COMMAND_SEC"
        done
    done

    sleep "$BETWEEN_PHASE_SEC"

    current_phase="status-after-on"
    for channel in "${CHANNEL_LIST[@]}"; do
        for node in "${NODE_LIST[@]}"; do
            run_cmd "status node $node channel $channel after ON" \
                "$VALVE_BIN" channel "$node" "$channel" status
            sleep "$BETWEEN_COMMAND_SEC"
        done
    done

    sleep "$BETWEEN_PHASE_SEC"

    current_phase="set-off"
    for channel in "${CHANNEL_LIST[@]}"; do
        for node in "${NODE_LIST[@]}"; do
            run_cmd "set node $node channel $channel OFF" \
                "$VALVE_BIN" set "$node" "$channel" off
            sleep "$BETWEEN_COMMAND_SEC"
        done
    done

    sleep "$BETWEEN_PHASE_SEC"

    current_phase="status-after-off"
    for channel in "${CHANNEL_LIST[@]}"; do
        for node in "${NODE_LIST[@]}"; do
            run_cmd "status node $node channel $channel after OFF" \
                "$VALVE_BIN" channel "$node" "$channel" status
            sleep "$BETWEEN_COMMAND_SEC"
        done
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
for channel in "${CHANNEL_LIST[@]}"; do
    for node in "${NODE_LIST[@]}"; do
        run_cmd "final status node $node channel $channel" \
            "$VALVE_BIN" channel "$node" "$channel" status
        sleep "$BETWEEN_COMMAND_SEC"
    done
done

if [[ "$fail_count" -eq 0 ]]; then
    exit 0
fi

exit 1
