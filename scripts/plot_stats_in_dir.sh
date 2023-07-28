#!/bin/bash

TO_PLOT="$1"

for LOGDIR in ${TO_PLOT}/*;
do
    printf "Plotting $LOGDIR\n"
    python analysis/plot_stats_vs_n_agents.py \
        --logdirs $LOGDIR
    printf "Done!\n\n"
done