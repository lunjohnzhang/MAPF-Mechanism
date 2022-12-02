#!/bin/bash

USAGE="Usage: bash scripts/run.sh OUT_DIR N_AGENTS SCEN_FILE MAP_FILE"

NAME="$1"
N_AGENTS="$2"
SCEN_FILE="$3"
MAP_FILE="$4"

if [ -z "${NAME}" ];
then
  echo "${USAGE}"
  exit 1
fi

if [ -z "${N_AGENTS}" ];
then
  echo "${USAGE}"
  exit 1
fi

if [ -z "${SCEN_FILE}" ];
then
  echo "${USAGE}"
  exit 1
fi

if [ -z "${MAP_FILE}" ];
then
  echo "${USAGE}"
  exit 1
fi

curr_time=`date +"%Y-%m-%d_%H-%M-%S"`

OUT_DIR_FULL="logs/$curr_time-${N_AGENTS}agents-$NAME"
mkdir -p $OUT_DIR_FULL

for w in $(seq 1.02 0.02 1.2)
do
    EXP_OUT_DIR="$OUT_DIR_FULL/w=$w"
    mkdir -p $EXP_OUT_DIR
    for i in {0..9}
    do
        ./build/eecbs \
            -m "$MAP_FILE" \
            -a "$SCEN_FILE" \
            -o "$EXP_OUT_DIR"/result.csv \
            --outputPaths="$EXP_OUT_DIR"/paths"_w=${w}_${i}".csv \
            -k "$N_AGENTS" \
            -t 60 \
            --suboptimality $w \
            --agentSuboptimality $w &
    done
done
