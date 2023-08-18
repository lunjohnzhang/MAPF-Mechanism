# Scalable Mechanism Design for Multi-Agent Path Finding

This repository is the official implementation of the paper **Scalable Mechanism Design for Multi-Agent Path Finding**. The repository builds on top of repositories of [EECBS](https://github.com/Jiaoyang-Li/EECBS), [PBS](https://github.com/Jiaoyang-Li/PBS), and [prioritized planning with ML](https://github.com/Jiaoyang-Li/Prioritized-Planning-with-ML).

The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake:

## Usage

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
./drone -m maps/random-32-32-20.map -a scens/random-32-32-20-random-1.scen -k 50 -t 60 --suboptimality=1.2 --algo ECBS
```
- algo: the MAPF algorithm. One of `CBS`, `ECBS`, `PBS`, and `PP`.
- map: the map file from the MAPF benchmark
- agents: the scenario file from the MAPF benchmark
- agentNum: the number of agents
- cutoffTime: the runtime limit
- saveStats: whether to save the search statistics
- savePath: whether to save the paths
- seed: global random seed
- nLayers: height of 3D map
- dummyStart: whether to create a dummy start node in single agent solver
- nRuns: number of random orders to run in `PP`.
- exhaustiveSearch: whether to use exhaustive search in `PBS`
- suboptimality: the suboptimality factor w for `ECBS`
- agentSuboptimality: per agent suboptimality factor for `ECBS`

You can find more details and explanations for all parameters with:

```
./drone --help
```

To test the code on more instances,
you can download the MAPF instances from the [MAPF benchmark](https://movingai.com/benchmarks/mapf/index.html).
In particular, the format of the scen files is explained [here](https://movingai.com/benchmarks/formats.html).
For a given number of agents k, the first k rows of the scen file are used to generate the k pairs of start and target locations.