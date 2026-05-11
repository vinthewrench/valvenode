#!/usr/bin/env bash
#
# test_ping_nodes.sh
#
# Repeatedly ping ValveNode nodes using the vnode test program.
#
# Usage:
#   ./test_ping_nodes.sh
#   ./test_ping_nodes.sh 1 2 3
#   VNODE=./build/vnode ./test_ping_nodes.sh 1 2
#

set -u

VNODE="${VNODE:-./vnode}"
COUNT="${COUNT:-20}"
DELAY="${DELAY:-0.25}"

if [[ $# -gt 0 ]]; then
    NODES=("$@")
else
    NODES=(1 3 4  5 6)
fi

if [[ ! -x "$VNODE" ]]; then
    echo "ERROR: vnode program not found or not executable: $VNODE"
    exit 1
fi

echo "ValveNode ping test"
echo "Program : $VNODE"
echo "Nodes   : ${NODES[*]}"
echo "Count   : $COUNT"
echo "Delay   : $DELAY sec"
echo

declare -A PASS
declare -A FAIL

for node in "${NODES[@]}"; do
    PASS["$node"]=0
    FAIL["$node"]=0
done

for ((i = 1; i <= COUNT; i++)); do
    echo "---- pass $i of $COUNT ----"

    for node in "${NODES[@]}"; do
        printf "node %s: " "$node"

        output="$("$VNODE" ping "$node" 2>&1)"
        rc=$?

        if [[ $rc -eq 0 ]]; then
            echo "PASS"
            PASS["$node"]=$((PASS["$node"] + 1))
        else
            echo "FAIL"
            FAIL["$node"]=$((FAIL["$node"] + 1))
            echo "  $output"
        fi

        sleep "$DELAY"
    done

    echo
done

echo "Summary"
echo "-------"

for node in "${NODES[@]}"; do
    total=$((PASS["$node"] + FAIL["$node"]))
    printf "node %s: pass=%d fail=%d total=%d\n" \
        "$node" "${PASS["$node"]}" "${FAIL["$node"]}" "$total"
done

