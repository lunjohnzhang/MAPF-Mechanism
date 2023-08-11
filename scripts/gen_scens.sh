for i in $(seq 1 10); do
    ./build/drone -m maps/random-32-32-20.map -a custom_scens/random-32-32-20-layer=1-$i.scen -t 120  --cost config/agent_costs/uniform/1000_$i.json --value config/agent_values/uniform/1000_$i.json --seed $i -k 1000 --nLayers 1 --algo PP1 --nRuns 1 --screen 1
done