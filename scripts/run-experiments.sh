#!/bin/sh

trap exit SIGTERM SIGHUP SIGINT

whca() {
    echo "=== WHCA* (window $1) ==="
    $experiments ../tmp/whca-$1 $2 $3 ../bin/opt/cli --scenario {} --algorithm whca --window $1
}

config() {
    agents=$1
    obstacles=$2

    experiments='./experiments.py --timeout 5 --threads 4 ../da-maps'

    echo "=== LRA* ==="
    $experiments ../tmp/lra $agents $obstacles ../bin/opt/cli --scenario {} --algorithm lra


    whca 5 $agents $obstacles
    whca 10 $agents $obstacles
    whca 15 $agents $obstacles
    whca 20 $agents $obstacles
}

config 5 0.01
config 5 0.1
config 50 0.01
config 50 0.1
