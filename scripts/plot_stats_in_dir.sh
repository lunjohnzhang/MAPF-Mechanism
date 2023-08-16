#!/bin/bash

TO_PLOT="$1"
ADD_LEGEND="$2"

if [ -z "${ADD_LEGEND}" ]; then
    ADD_LEGEND="True"
fi


for LOGDIR in ${TO_PLOT}/*;
do
    printf "Plotting $LOGDIR\n"
    python analysis/plot_stats_vs_n_agents.py \
        --logdirs $LOGDIR \
        --add-legend $ADD_LEGEND &
done

wait
printf "Done!\n\n"