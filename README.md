# Scalable Mechanism Design for Multi-Agent Path Finding

This repository is the official implementation of the paper **Scalable Mechanism Design for Multi-Agent Path Finding**, accepted to IJCAI 2024. The repository builds on top of repositories of [EECBS](https://github.com/Jiaoyang-Li/EECBS), [PBS](https://github.com/Jiaoyang-Li/PBS), and [prioritized planning with ML](https://github.com/Jiaoyang-Li/Prioritized-Planning-with-ML).


## Installation

The code requires the external libraries [boost](https://www.boost.org/) and [parlaylib](https://github.com/cmuparlay/parlaylib).

### Install boost

If you are using Ubuntu, you can install it simply by

```shell script
sudo apt install libboost-all-dev
```

Another easy way of installing the boost library is to install anaconda/miniconda and then

```shell script
conda install -c anaconda libboost
```

which works for a variety of [systems](https://anaconda.org/anaconda/libboost)
(including linux, osx, and win).

If neither of the above method works, you can also follow the instructions
on the [boost](https://www.boost.org/) website and install it manually.

### Install parlaylib

To install parlaylib, please follow the instruction in their [documentation](https://cmuparlay.github.io/parlaylib/installation.html).

### Compile and Run

After you installed boost and parlaylib, and downloaded the source code, go into the directory of the source code and compile it with CMake:

```shell script
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```

Then, you are able to run the code:

```
./build/drone --map maps/random-32-32-20.map \
    --agents custom_scens/random-32-32-20-my-1.scen \
    --cost config/agent_costs/uniform/1000_1.json \
    --value config/agent_values/uniform/1000_1.json \
    --cutoffTime 120 --seed 1 --agentNum 1500 --nLayers 1 \
    --algo PP1 --nRuns 1 --screen 1
```

- map: the map file from the MAPF benchmark
- agents: the scenario file from the MAPF benchmark
- cost: config file for costs
- value: config file for values
- agentNum: the number of agents
- cutoffTime: the runtime limit
- seed: global random seed
- nLayers: height of 3D map
- nRuns: number of random orders to run in `PP`.
- algo: the MAPF algorithm. One of `CBS`, `PBS`, and `PP`.
- nRuns: number of times to run in Monte Carlo PP. i.e. if `nRuns=1`, MCPP becomes naive PP (aka first come first serve).
- screen: screen level of the program
- savePath: whether to save the paths, default to True
- dummyStart: whether to create a dummy start node in single agent solver, default to True
- exhaustiveSearch: whether to use exhaustive search in `PBS`, default to True

You can find more details and explanations for all parameters with:

```
./drone --help
```

To test the code on more instances,
you can download the MAPF instances from the [MAPF benchmark](https://movingai.com/benchmarks/mapf/index.html).
In particular, the format of the scen files is explained [here](https://movingai.com/benchmarks/formats.html).
For a given number of agents k, the first k rows of the scen file are used to generate the k pairs of start and target locations.