#!/bin/bash

USAGE="Usage: bash scripts/run.sh ALGO N_AGENTS SCEN_FILE MAP_FILE COST_FILE VALUE_FILE N_LAYERS N_RUNS N_SIM N_CORES ROOT_LOG"

algos=(
    "PP"
    "PP1"
    "PBS"
    "ECBS"
    "CBS"
)

SCEN_FILE="$1"  # scene file
MAP_FILE="$2"   # map file
COST_MODE="$3"  # cost config
VALUE_MODE="$4" # value config
N_LAYERS="$5"   # number of layers
N_RUNS="$6"     # number of runs for Monte Carlo PP
N_SIM="$7"      # number of simulations to run
N_CORES="$8"    # max number of cores available
N_AGENTS_MIN="$9"
N_AGENTS_STEP="${10}"
N_AGENTS_MAX="${11}"

if [ -z "${SCEN_FILE}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${MAP_FILE}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${COST_MODE}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${VALUE_MODE}" ]; then
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

if [ -z "${N_AGENTS_MIN}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${N_AGENTS_STEP}" ]; then
    echo "${USAGE}"
    exit 1
fi

if [ -z "${N_AGENTS_MAX}" ]; then
    echo "${USAGE}"
    exit 1
fi

# Create logdir
map_file_base=$(basename "$MAP_FILE")
map_file_name="${map_file_base%.*}"

logdir="logs/$(date +'%Y-%m-%d_%H-%M-%S')_${map_file_name}_layer=${N_LAYERS}"
mkdir -p $logdir

exit

# Ref: https://unix.stackexchange.com/questions/103920/parallelize-a-bash-for-loop/436713#436713
for algo in "${algos[@]}"; do
    echo $algo
    logdir_algo="${logdir}/$algo"
    mkdir -p $logdir_algo
    for n_agent in $(seq $N_AGENTS_MIN $N_AGENTS_STEP $N_AGENTS_MAX); do
        for i in $(seq 1 $N_SIM); do
            (
                echo "starting task $i.."
                ./build/drone \
                    -m "$MAP_FILE" \
                    -a "$SCEN_FILE" \
                    -k "$n_agent" \
                    -t 120 \
                    --cost config/agent_costs/${COST_MODE}/1000_$i.json \
                    --value config/agent_values/${VALUE_MODE}/1000_$i.json \
                    --algo $algo \
                    --suboptimality 1.05 \
                    --nLayers $N_LAYERS \
                    --nRuns $N_RUNS \
                    --seed $i \
                    --screen 0 \
                    --root_logdir $logdir_algo
                echo "Done task $i"
            ) &

            # allow to execute up to $N jobs in parallel
            if [[ $(jobs -r -p | wc -l) -ge $N_CORES ]]; then
                # now there are $N jobs already running, so wait here for any
                # job to be finished so there is a place to start next one.
                wait -n
            fi
        done
    done
done

# no more jobs to be started but wait for pending jobs
# (all need to be finished)
wait

echo "all done"
