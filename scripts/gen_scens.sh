MAP_NAME="$1"
N_AGENTS="$2"


for i in $(seq 1 10); do
    ./build/drone -m maps/${MAP_NAME}.map -a custom_scens/${MAP_NAME}-my-$i.scen -t 120  --cost config/agent_costs/uniform/1000_$i.json --value config/agent_values/uniform/1000_$i.json --seed $i -k ${N_AGENTS} --nLayers 1 --algo PP1 --nRuns 1 --screen 1 &
done