#!/bin/bash

USAGE="Usage: bash scripts/run.sh ALGO N_AGENTS SCEN_FILE MAP_FILE COST_FILE VALUE_FILE N_LAYERS N_RUNS N_SIM N_CORES ROOT_LOG"

algos=(
    "PP"
    "PP1"
    "PBS"
    "ECBS"
    "CBS"
)

function get_dirname() {
    full_path=$1

    # Remove trailing slash
    dirname=${full_path%/}

    # Extract only the directory name
    dirname=${dirname##*/}

    echo $dirname
}

function split_str() {
    # Change the Internal Field Separator
    str=$1
    IFS=$2

    # Split the exp dir name to get the params
    read -ra PARTS <<<"$str"

    echo "${PARTS[2]}$IFS${PARTS[3]}$IFS${PARTS[4]}"
}

# function contains() {
#     local e match="$1"
#     shift
#     for e; do [[ "$e" == "$match" ]] && return 0; done
#     return 1
# }

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
RELOAD_DIR="${12}"

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

# Array to store dynamic params for each experiment
all_algos_exp=()
all_n_agents_exp=()
all_seeds_exp=()
all_root_logdir=()

if [ -z "${RELOAD_DIR}" ]; then
    echo "New experiment"
    # Create new logdir
    map_file_base=$(basename "$MAP_FILE")
    map_file_name="${map_file_base%.*}"

    logdir="logs/$(date +'%Y-%m-%d_%H-%M-%S')_${map_file_name}_layer=${N_LAYERS}"
    mkdir -p $logdir
    for i in $(seq 1 $N_SIM); do
        for algo in "${algos[@]}"; do
            for n_agent in $(seq $N_AGENTS_MIN $N_AGENTS_STEP $N_AGENTS_MAX); do
                logdir_algo="${logdir}/$algo"
                mkdir -p $logdir_algo
                # echo "$algo $n_agent $i $logdir_algo"
                all_algos_exp+=("$algo")
                all_n_agents_exp+=("$n_agent")
                all_seeds_exp+=("$i")
                all_root_logdir+=("$logdir_algo")
            done
        done
    done
else
    echo "Reloading from ${RELOAD_DIR}"
    logdir=$RELOAD_DIR

    # Check which experiments are done
    declare -A have_run
    for algo_dir in "$RELOAD_DIR"/*; do
        algo_name=$(get_dirname "$algo_dir")

        # Check if the item is a directory
        if [ -d "$algo_dir" ]; then
            for exp_dir in "$algo_dir"/*; do
                if [ -d "$exp_dir" ]; then
                    # Check if result.json exist
                    exp_dir_name=$(get_dirname "$exp_dir")
                    exp_fingerprint=$(split_str $exp_dir_name "_")
                    # echo $exp_fingerprint
                    if [ -f "$exp_dir/result.json" ]; then
                        have_run[$exp_fingerprint]=1
                    else
                        # Remove the original exp log directory
                        echo did not run ${exp_fingerprint}, removing...
                        rm -rf $exp_dir
                    fi
                fi
            done
        fi
    done

    # Iterate through and find experiments to rerun
    for i in $(seq 1 $N_SIM); do
        for algo in "${algos[@]}"; do
            for n_agent in $(seq $N_AGENTS_MIN $N_AGENTS_STEP $N_AGENTS_MAX); do
                exp_fingerprint="${algo}_k=${n_agent}_seed=${i}"

                if [[ ! -n ${have_run[$exp_fingerprint]} ]]; then
                    echo "Have_run does not contain $exp_fingerprint"
                    logdir_algo="${logdir}/$algo"
                    mkdir -p $logdir_algo
                    all_algos_exp+=("$algo")
                    all_n_agents_exp+=("$n_agent")
                    all_seeds_exp+=("$i")
                    all_root_logdir+=("$logdir_algo")
                fi
            done
        done
    done
fi

# Ref: https://unix.stackexchange.com/questions/103920/parallelize-a-bash-for-loop/436713#436713

for ((i = 0; i < ${#all_algos_exp[@]}; i++)); do
    (
        echo "starting task $i.."
        ./build/drone \
            -m $MAP_FILE \
            -a $SCEN_FILE \
            -k ${all_n_agents_exp[$i]} \
            -t 120 \
            --cost config/agent_costs/${COST_MODE}/1000_${all_seeds_exp[$i]}.json \
            --value config/agent_values/${VALUE_MODE}/1000_${all_seeds_exp[$i]}.json \
            --algo ${all_algos_exp[$i]} \
            --suboptimality 1.05 \
            --nLayers $N_LAYERS \
            --nRuns $N_RUNS \
            --seed ${all_seeds_exp[$i]} \
            --screen 0 \
            --root_logdir ${all_root_logdir[$i]}
        echo "Done task $i"
    ) &

    # allow to execute up to $N jobs in parallel
    if [[ $(jobs -r -p | wc -l) -ge $N_CORES ]]; then
        # now there are $N jobs already running, so wait here for any
        # job to be finished so there is a place to start next one.
        wait -n
    fi
    # echo "${all_algos_exp[$i]} ${all_n_agents_exp[$i]} ${all_seeds_exp[$i]} ${all_root_logdir[$i]}"
done

# for algo in "${algos[@]}"; do
#     echo $algo
#     logdir_algo="${logdir}/$algo"
#     mkdir -p $logdir_algo
#     for n_agent in $(seq $N_AGENTS_MIN $N_AGENTS_STEP $N_AGENTS_MAX); do
#         for i in $(seq 1 $N_SIM); do
#             (
#                 echo "starting task $i.."
#                 ./build/drone \
#                     -m "$MAP_FILE" \
#                     -a "$SCEN_FILE" \
#                     -k "$n_agent" \
#                     -t 120 \
#                     --cost config/agent_costs/${COST_MODE}/1000_$i.json \
#                     --value config/agent_values/${VALUE_MODE}/1000_$i.json \
#                     --algo $algo \
#                     --suboptimality 1.05 \
#                     --nLayers $N_LAYERS \
#                     --nRuns $N_RUNS \
#                     --seed $i \
#                     --screen 0 \
#                     --root_logdir $logdir_algo
#                 echo "Done task $i"
#             ) &

#             # allow to execute up to $N jobs in parallel
#             if [[ $(jobs -r -p | wc -l) -ge $N_CORES ]]; then
#                 # now there are $N jobs already running, so wait here for any
#                 # job to be finished so there is a place to start next one.
#                 wait -n
#             fi
#         done
#     done
# done

# no more jobs to be started but wait for pending jobs
# (all need to be finished)
wait

echo "all done"
