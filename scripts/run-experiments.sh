#!/bin/sh

trap exit SIGTERM SIGHUP SIGINT

experiments='./experiments.py --timeout 5 --threads 4 ../da-maps'

echo "=== LRA* ==="
$experiments ../tmp/lra ../build/release/cli/cli --scenario {} --algorithm lra

whca() {
    echo "=== WHCA* (window $1) ==="
    $experiments ../tmp/whca-$1 ../build/release/cli/cli --scenario {} --algorithm whca --window $1

}

whca 5
whca 10
whca 15
whca 20
