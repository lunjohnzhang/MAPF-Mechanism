#!/bin/bash

USAGE="Usage: bash scripts/run.sh ALGO N_AGENTS SCEN_FILE MAP_FILE N_LAYERS N_RUNS N_SIM N_CORES"

ALGO="$1"
N_AGENTS="$2"
SCEN_FILE="$3"
MAP_FILE="$4"
N_LAYERS="$5"
N_RUNS="$6"
N_SIM="$7"
N_CORES="$8"

if [ -z "${ALGO}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${N_AGENTS}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${SCEN_FILE}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${MAP_FILE}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${N_LAYERS}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${N_RUNS}" ]; then
    echo "${USAGE}"
    exit 1
fi


if [ -z "${N_SIM}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${N_CORES}" ]; then
    echo "${USAGE}"
    exit 1
fi

# for i in $(seq 1 $N_SIM); do
#     ./build/drone \
#         -m "$MAP_FILE" \
#         -a "$SCEN_FILE" \
#         -k "$N_AGENTS" \
#         -t 120 \
#         --suboptimality $w \
#         --nLayers $N_LAYERS \
#         --nRuns $N_RUNS \
#         --screen $i &
# done


# Ref: https://unix.stackexchange.com/questions/103920/parallelize-a-bash-for-loop/436713#436713
for i in $(seq 1 $N_SIM); do
    (
        echo "starting task $i.."
        ./build/drone \
            -m "$MAP_FILE" \
            -a "$SCEN_FILE" \
            -k "$N_AGENTS" \
            -t 120 \
            --algo $ALGO \
            --suboptimality 1.05 \
            --nLayers $N_LAYERS \
            --nRuns $N_RUNS \
            --seed $i \
            --screen 0
        echo "Done task $i"
    ) &

    # allow to execute up to $N jobs in parallel
    if [[ $(jobs -r -p | wc -l) -ge $N_CORES ]]; then
        # now there are $N jobs already running, so wait here for any job
        # to be finished so there is a place to start next one.
        wait -n
    fi

done

# no more jobs to be started but wait for pending jobs
# (all need to be finished)
wait

echo "all done"