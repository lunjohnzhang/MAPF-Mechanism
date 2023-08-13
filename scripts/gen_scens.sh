for i in $(seq 1 10); do
    ./build/drone -m maps/Paris_1_256.map -a custom_scens/Paris_1_256-my-$i.scen -t 120  --cost config/agent_costs/uniform/1000_$i.json --value config/agent_values/uniform/1000_$i.json --seed $i -k 20000 --nLayers 1 --algo PP1 --nRuns 1 --screen 1 &
done