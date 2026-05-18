#!/usr/bin/env bash
#
# version_scan.sh
#
# Discover ValveNode slaves with `valve who`, then query firmware versions.
#
# Pattern:
#   power on
#   who
#   version each discovered node
#   power off
#
# Usage:
#   chmod +x version_scan.sh
#   ./version_scan.sh
#
# Optional:
#   VERBOSE=1 ./version_scan.sh
#   VALVE_BIN=/path/to/valve ./version_scan.sh
#   CMD_TIMEOUT_SEC=5 ./version_scan.sh
#   POWER_SETTLE_SEC=1 ./version_scan.sh
#   BETWEEN_COMMAND_SEC=0.10 ./version_scan.sh
#
# Valve binary search priority:
#   1. VALVE_BIN override
#   2. ./valve in current directory
#   3. valve from PATH

set -u

if [[ -n "${VALVE_BIN:-}" ]]; then
    VALVE_BIN="$VALVE_BIN"
elif [[ -x "./valve" ]]; then
    VALVE_BIN="./valve"
else
    VALVE_BIN="$(command -v valve || true)"
fi

VERBOSE="${VERBOSE:-0}"
CMD_TIMEOUT_SEC="${CMD_TIMEOUT_SEC:-8}"
POWER_SETTLE_SEC="${POWER_SETTLE_SEC:-5}"
BETWEEN_COMMAND_SEC="${BETWEEN_COMMAND_SEC:-0.10}"

TMP_OUT="/tmp/version_scan_last.out"
TMP_ERR="/tmp/version_scan_last.err"
WHO_OUT="/tmp/version_scan_who.out"
WHO_ERR="/tmp/version_scan_who.err"

pass_count=0
fail_count=0
timeout_count=0
interrupted=0
cleaned_up=0

timestamp()
{
    date "+%Y-%m-%d %H:%M:%S"
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

    rm -f "$TMP_OUT" "$TMP_ERR" "$WHO_OUT" "$WHO_ERR"
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

capture_file_output()
{
    local out_file="$1"
    local err_file="$2"

    if [[ -s "$out_file" ]]; then
        tr '\n' ' ' < "$out_file"
    elif [[ -s "$err_file" ]]; then
        tr '\n' ' ' < "$err_file"
    fi
}

discover_nodes()
{
    : >"$WHO_OUT"
    : >"$WHO_ERR"

    if [[ "$VERBOSE" != "0" ]]; then
        echo
        echo "==> who"
        echo "+ timeout $CMD_TIMEOUT_SEC $VALVE_BIN who"
        timeout "$CMD_TIMEOUT_SEC" "$VALVE_BIN" who
    else
        timeout "$CMD_TIMEOUT_SEC" "$VALVE_BIN" who >"$WHO_OUT" 2>"$WHO_ERR"
    fi

    local rc=$?

    if [[ "$VERBOSE" != "0" ]]; then
        return "$rc"
    fi

    return "$rc"
}

parse_nodes_from_who()
{
    awk '
        /^[[:space:]]+[0-9]+[[:space:]]*$/ {
            print $1
        }
    ' "$WHO_OUT"
}

run_version()
{
    local node="$1"

    : >"$TMP_OUT"
    : >"$TMP_ERR"

    if [[ "$VERBOSE" != "0" ]]; then
        echo
        echo "==> version node $node"
        echo "+ timeout $CMD_TIMEOUT_SEC $VALVE_BIN version $node"
        timeout "$CMD_TIMEOUT_SEC" "$VALVE_BIN" version "$node"
    else
        timeout "$CMD_TIMEOUT_SEC" "$VALVE_BIN" version "$node" >"$TMP_OUT" 2>"$TMP_ERR"
    fi

    return $?
}

echo "Valve bus version scan"
echo "  valve binary:        $VALVE_BIN"
echo "  discovery:           valve who"
echo "  verbose:             $VERBOSE"
echo "  command timeout sec: $CMD_TIMEOUT_SEC"
echo

echo "Turning field power on..."
timeout "$CMD_TIMEOUT_SEC" "$VALVE_BIN" power on >/dev/null 2>&1 || {
    echo "ERROR: failed to turn field power on"
    exit 1
}

sleep "$POWER_SETTLE_SEC"

echo "Discovering nodes..."
discover_nodes
who_rc=$?

if [[ "$who_rc" -ne 0 ]]; then
    who_output="$(capture_file_output "$WHO_OUT" "$WHO_ERR")"
    echo "ERROR: valve who failed: rc=$who_rc"
    echo "$who_output"
    exit 1
fi

mapfile -t NODE_LIST < <(parse_nodes_from_who)

if [[ "${#NODE_LIST[@]}" -lt 1 ]]; then
    echo "ERROR: no nodes discovered from valve who"
    echo
    echo "Raw who output:"
    cat "$WHO_OUT"
    exit 1
fi

echo "Discovered nodes: ${NODE_LIST[*]}"

echo
printf "%-8s %-8s %s\n" "Node" "Result" "Version / Output"
printf "%-8s %-8s %s\n" "----" "------" "----------------"

for node in "${NODE_LIST[@]}"; do
    run_version "$node"
    rc=$?

    output="$(capture_file_output "$TMP_OUT" "$TMP_ERR")"

    if [[ "$rc" -eq 0 ]]; then
        pass_count=$((pass_count + 1))
        printf "%-8s %-8s %s\n" "$node" "OK" "$output"
    else
        fail_count=$((fail_count + 1))

        if [[ "$rc" -eq 124 ]]; then
            timeout_count=$((timeout_count + 1))
            printf "%-8s %-8s %s\n" "$node" "TIMEOUT" "$output"
        else
            printf "%-8s %-8s rc=%s %s\n" "$node" "FAIL" "$rc" "$output"
        fi
    fi

    sleep "$BETWEEN_COMMAND_SEC"
done

echo
echo "============================================================"
echo "Version scan summary"
echo "============================================================"
echo "  nodes discovered:    ${NODE_LIST[*]}"
echo "  passes:              $pass_count"
echo "  fails:               $fail_count"
echo "  command timeouts:    $timeout_count"
echo "  interrupted:         $([[ "$interrupted" -ne 0 ]] && echo yes || echo no)"

if [[ "$fail_count" -eq 0 ]]; then
    exit 0
fi

exit 1
