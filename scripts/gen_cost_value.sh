#!/bin/bash

COST_MODE="$1"  # cost config
VALUE_MODE="$2" # value config
N_AGENTS_MIN="$3"
N_AGENTS_STEP="$4"
N_AGENTS_MAX="$5"

for n_agent in $(seq $N_AGENTS_MIN $N_AGENTS_STEP $N_AGENTS_MAX); do
    python analysis/gen_cost_value.py \
        --n_agent $n_agent \
        --cost_mode $COST_MODE \
        --value_mode $VALUE_MODE
done
