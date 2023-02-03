for i in {0..9}
    do
        ./build/eecbs -m maps/Paris_1_256.map -a scens/Paris_1_256-random-1.scen -o logs/test-"$i".csv --outputPaths=logs/500_agents-"$i".csv -k 500 -t 60 --suboptimality=1.02 --agentSuboptimality 1.02 --nLayers 10
    done