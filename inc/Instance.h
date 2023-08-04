#pragma once
#include "common.h"

// Currently only works for undirected unweighted 4-nighbor grids
class Instance
{
public:
    int num_of_cols;
    int num_of_rows;
    int num_of_layers;
    int map_size;

    // enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };
    // // MOVE_COUNT is the enum's size

    Instance() {}
    Instance(const string& map_fname, const string& agent_fname, const int seed,
             const string& cost_config, const string& value_config,
             int num_of_agents = 0, int num_of_rows = 0, int num_of_cols = 0,
             int num_of_layers = 0, int num_of_obstacles = 0,
             int warehouse_width = 0);
    Instance(const Instance& other)
        : num_of_cols(other.num_of_cols),
          num_of_rows(other.num_of_rows),
          num_of_layers(other.num_of_layers),
          map_size(other.map_size),
          num_of_agents(other.num_of_agents),
          start_locations(other.start_locations),
          goal_locations(other.goal_locations),
          seed(other.seed),
          gen(other.gen),
          cost_config(other.cost_config),
          costs(other.costs),
          cost_mode(other.cost_mode),
          value_config(other.value_config),
          values(other.values),
          value_mode(other.value_mode),
          my_map(other.my_map),
          agent_fname(other.agent_fname),
          map_fname(other.map_fname),
          distance_matrix(other.distance_matrix)
    {
    }

    void printAgents() const;

    inline bool isObstacle(int loc) const { return my_map[loc]; }
    inline bool validMove(int curr, int next) const;
    list<int> getNeighbors(int curr) const;

    inline int linearizeCoordinate(int row, int col, int layer) const
    {
        // return (this->num_of_cols * row + col);
        return (this->num_of_layers * this->num_of_cols * row +
                this->num_of_layers * col + layer);
    }
    inline int getRowCoordinate(int id) const
    {
        return id / (this->num_of_cols * this->num_of_layers);
    }
    inline int getColCoordinate(int id) const
    {
        return (id / this->num_of_layers) % this->num_of_cols;
    }
    inline int getLayerCoordinate(int id) const
    {
        return id % this->num_of_layers;
    }
    inline tuple<int, int, int> getCoordinate(int id) const
    {
        return make_tuple(getRowCoordinate(id), getColCoordinate(id),
                          getLayerCoordinate(id));
    }
    inline int getCols() const { return num_of_cols; }

    inline int getManhattanDistance(int loc1, int loc2) const
    {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc1_z = getLayerCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        int loc2_z = getLayerCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y) +
               abs(loc1_z - loc2_z);
    }

    inline int getManhattanDistance(const tuple<int, int, int>& loc1,
                                    const tuple<int, int, int>& loc2) const
    {
        return abs(get<0>(loc1) - get<0>(loc2)) +
               abs(get<1>(loc1) - get<1>(loc2)) +
               abs(get<2>(loc1) - get<2>(loc2));
        // return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

    // In how many directions the agent can move?
    int getDegree(int curr) const;

    int getDefaultNumberOfAgents() const { return num_of_agents; }

    int num_of_agents;
    vector<int> start_locations;
    vector<int> goal_locations;

    int seed;
    mt19937 gen;

    // Cost of each agent
    string cost_config;
    vector<double> costs;
    string cost_mode;

    // Value of each agent
    string value_config;
    vector<double> values;
    string value_mode;

    // Keep only the global agents and initialize the id_map.
    map<int, int> initPartialInstance(set<int> global_agents);

    const vector<int>* getDistances(int root_location);

    void saveAgentProfile(boost::filesystem::path filename);

    vector<tuple<int, int, int>> convertAgentLocations(
        vector<int> locations) const;

private:
    // int moves_offset[MOVE_COUNT];
    vector<bool> my_map;
    string map_fname;
    string agent_fname;
    unordered_map<int, vector<int>> distance_matrix;

    bool loadMap();
    void printMap() const;
    void saveMap() const;

    bool loadAgents();
    void saveAgents() const;

    // initialize new [rows x cols] map with random obstacles
    void generateConnectedRandomGrid(int rows, int cols, int layers,
                                     int obstacles);
    void generateRandomAgents(int warehouse_width);

    // add this obsatcle only if the map is still connected
    bool addObstacle(int obstacle);

    // run BFS to find a path between start and goal, return true if a path
    // exists.
    bool isConnected(int start, int goal);

    int randomWalk(int loc, int steps) const;

    // Class  SingleAgentSolver can access private members of Node
    friend class SingleAgentSolver;
};
