#include "Instance.h"

#include <algorithm>  // std::shuffle
#include <boost/tokenizer.hpp>
#include <chrono>  // std::chrono::system_clock
#include <random>  // std::default_random_engine

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(const string& map_fname, const string& agent_fname,
                   const int seed, const string& cost_config,
                   const string& value_config, int num_of_agents,
                   int num_of_rows, int num_of_cols, int num_of_layers,
                   int num_of_obstacles, int warehouse_width)
    : map_fname(map_fname),
      agent_fname(agent_fname),
      seed(seed),
      cost_config(cost_config),
      value_config(value_config),
      num_of_agents(num_of_agents),
      num_of_layers(num_of_layers)
{
    bool succ = loadMap();
    if (!succ)
    {
        // generate random grid
        if (num_of_rows > 0 && num_of_cols > 0 && num_of_layers > 0 &&
            num_of_obstacles >= 0 &&
            num_of_obstacles < num_of_rows * num_of_cols)
        {
            generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_layers,
                                        num_of_obstacles);
            saveMap();
        }
        else
        {
            cerr << "Map file " << map_fname << " not found." << endl;
            exit(-1);
        }
    }

    succ = loadAgents();
    if (!succ)
    {
        if (num_of_agents > 0)
        {
            generateRandomAgents(warehouse_width);
            saveAgents();
        }
        else
        {
            cerr << "Agent file " << agent_fname << " not found." << endl;
            exit(-1);
        }
    }

    // Read in costs and values
    std::ifstream cost_f(this->cost_config);
    json cost_config_json = json::parse(cost_f);
    std::ifstream value_f(this->value_config);
    json value_config_json = json::parse(value_f);

    this->cost_mode = cost_config_json["mode"];
    this->value_mode = value_config_json["mode"];

    this->costs.resize(num_of_agents);
    this->values.resize(num_of_agents);

    // Compute heuristic here to compute the costs.
    // TODO: We compute heuristic multiple times (here and in single agent
    // solver). Refractor to compute only once.
    for (int i = 0; i < this->num_of_agents; i++)
    {
        getDistances(goal_locations[i], start_locations[i], goal_locations[i]);
    }

    for (int i = 0; i < this->num_of_agents; i++)
    {
        this->values[i] = value_config_json["vals"][i];

        // Compute cost as cost =
        // costFactor*value/shortestPathWithoutConstraints
        this->costs[i] =
            (double)cost_config_json["vals"][i] * this->values[i] /
            (double)distance_matrix[goal_locations[i]][start_locations[i]];
    }
    // Cleared because dummy start loc is not properly set here so the
    // heuristic of dummy start loc is wrong.
    // TODO: sort out deadlock between dummy start loc setting and heuristic
    // computing.
    distance_matrix.clear();
}

int Instance::randomWalk(int curr, int steps) const
{
    for (int walk = 0; walk < steps; walk++)
    {
        list<int> l = getNeighbors(curr);
        vector<int> next_locations(l.cbegin(), l.cend());
        auto rng = std::default_random_engine{};
        std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
        for (int next : next_locations)
        {
            if (validMove(curr, next))
            {
                curr = next;
                break;
            }
        }
    }
    return curr;
}

void Instance::generateRandomAgents(int warehouse_width)
{
    cout << "Generate " << num_of_agents << " random start and goal locations "
         << endl;
    vector<bool> starts(map_size, false);
    vector<bool> goals(map_size, false);
    start_locations.resize(num_of_agents);
    goal_locations.resize(num_of_agents);

    if (warehouse_width == 0)  // Generate agents randomly
    {
        // Choose random start locations
        int k = 0;
        while (k < num_of_agents)
        {
            int x = rand() % num_of_rows, y = rand() % num_of_cols;
            // Always start at the ground level
            int start = linearizeCoordinate(x, y, 0);
            // if (my_map[start] || starts[start])
            //     continue;
            // We can have multiple agents start from the same place.
            if (my_map[start])
                continue;

            // update start
            start_locations[k] = start;
            // starts[start] = true;

            // find goal
            bool flag = false;
            x = rand() % num_of_rows;
            y = rand() % num_of_cols;
            // Always end at the ground level
            int goal = linearizeCoordinate(x, y, 0);
            // while (my_map[goal] || goals[goal])
            //     goal = rand() % map_size;
            // We can have multiple agents end at the same place.
            while (my_map[goal] || start == goal)
            {
                x = rand() % num_of_rows;
                y = rand() % num_of_cols;
                // Always end at the ground level
                goal = linearizeCoordinate(x, y, 0);
            }

            // update goal
            goal_locations[k] = goal;
            // goals[goal] = true;

            k++;
        }
    }
    else  // Generate agents for warehouse scenario
    {
        // Choose random start locations
        int k = 0;
        while (k < num_of_agents)
        {
            int x = rand() % num_of_rows, y = rand() % warehouse_width,
                z = rand() % num_of_layers;
            if (k % 2 == 0)
                y = num_of_cols - y - 1;
            int start = linearizeCoordinate(x, y, z);
            if (starts[start])
                continue;
            // update start
            start_locations[k] = start;
            starts[start] = true;

            k++;
        }
        // Choose random goal locations
        k = 0;
        while (k < num_of_agents)
        {
            int x = rand() % num_of_rows, y = rand() % warehouse_width,
                z = rand() % num_of_layers;
            if (k % 2 == 1)
                y = num_of_cols - y - 1;
            int goal = linearizeCoordinate(x, y, z);
            if (goals[goal])
                continue;
            // update goal
            goal_locations[k] = goal;
            goals[goal] = true;
            k++;
        }
    }
}

bool Instance::validMove(int curr, int next) const
{
    if (next < 0 || next >= map_size ||
        // Go from non-dummy start to dummy start: not valid!
        (curr != GLOBAL_VAR::dummy_start_loc &&
         next == GLOBAL_VAR::dummy_start_loc))
        return false;
    if (my_map[next])
        return false;
    return getManhattanDistance(curr, next) < 2;
}

bool Instance::addObstacle(int obstacle)
{
    if (my_map[obstacle])
        return false;
    my_map[obstacle] = true;
    int obstacle_x = getRowCoordinate(obstacle);
    int obstacle_y = getColCoordinate(obstacle);
    int obstacle_z = getLayerCoordinate(obstacle);
    int x[6] = {
        obstacle_x,     obstacle_x + 1, obstacle_x,
        obstacle_x - 1, obstacle_x,     obstacle_x,
    };
    int y[6] = {
        obstacle_y - 1, obstacle_y, obstacle_y + 1,
        obstacle_y,     obstacle_y, obstacle_y,
    };
    int z[6] = {
        obstacle_z, obstacle_z,     obstacle_z,
        obstacle_z, obstacle_z + 1, obstacle_z - 1,
    };
    int start = 0;
    int goal = 1;
    while (start < 5 && goal < 6)
    {
        if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 ||
            y[start] >= num_of_cols || z[start] < 0 ||
            z[start] >= num_of_layers ||
            my_map[linearizeCoordinate(x[start], y[start], z[start])])
            start++;
        else if (goal <= start)
            goal = start + 1;
        else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 ||
                 y[goal] >= num_of_cols || z[goal] < 0 ||
                 z[goal] >= num_of_layers ||
                 my_map[linearizeCoordinate(x[goal], y[goal], z[goal])])
            goal++;
        // cannot find a path from start to goal
        else if (isConnected(linearizeCoordinate(x[start], y[start], z[start]),
                             linearizeCoordinate(x[goal], y[goal], z[goal])))
        {
            start = goal;
            goal++;
        }
        else
        {
            my_map[obstacle] = false;
            return false;
        }
    }
    return true;
}

bool Instance::isConnected(int start, int goal)
{
    std::queue<int> open;
    vector<bool> closed(map_size, false);
    open.push(start);
    closed[start] = true;
    while (!open.empty())
    {
        int curr = open.front();
        open.pop();
        if (curr == goal)
            return true;
        for (int next : getNeighbors(curr))
        {
            if (closed[next])
                continue;
            open.push(next);
            closed[next] = true;
        }
    }
    return false;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int layers,
                                           int obstacles)
{
    cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles
         << " obstacles. " << endl;
    int i, j;
    num_of_rows = rows + 2;
    num_of_cols = cols + 2;
    num_of_layers = layers + 2;
    map_size = num_of_rows * num_of_cols * num_of_layers;
    my_map.resize(map_size, false);
    // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
    /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
    moves_offset[Instance::valid_moves_t::EAST] = 1;
    moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
    moves_offset[Instance::valid_moves_t::WEST] = -1;*/

    // add padding
    for (int k = 1; k < num_of_layers - 1; k++)
    {
        i = 0;
        for (j = 0; j < num_of_cols; j++)
            my_map[linearizeCoordinate(i, j, k)] = true;
        i = num_of_rows - 1;
        for (j = 0; j < num_of_cols; j++)
            my_map[linearizeCoordinate(i, j, k)] = true;
        j = 0;
        for (i = 0; i < num_of_rows; i++)
            my_map[linearizeCoordinate(i, j, k)] = true;
        j = num_of_cols - 1;
        for (i = 0; i < num_of_rows; i++)
            my_map[linearizeCoordinate(i, j, k)] = true;
    }
    for (int i = 0; i < num_of_cols; i++)
    {
        for (int j = 0; j < num_of_cols; j++)
        {
            my_map[linearizeCoordinate(i, j, 0)] = true;
            my_map[linearizeCoordinate(i, j, num_of_layers - 1)] = true;
        }
    }

    // add obstacles uniformly at random
    i = 0;
    while (i < obstacles)
    {
        int loc = rand() % map_size;
        if (addObstacle(loc))
        {
            printMap();
            i++;
        }
    }
}

bool Instance::loadMap()
{
    using namespace boost;
    using namespace std;
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open())
        return false;
    string line;
    tokenizer<char_separator<char>>::iterator beg;
    getline(myfile, line);
    if (line[0] == 't')  // Nathan's benchmark
    {
        char_separator<char> sep(" ");
        getline(myfile, line);
        tokenizer<char_separator<char>> tok(line, sep);
        beg = tok.begin();
        beg++;
        num_of_rows = atoi((*beg).c_str());  // read number of rows
        getline(myfile, line);
        tokenizer<char_separator<char>> tok2(line, sep);
        beg = tok2.begin();
        beg++;
        num_of_cols = atoi((*beg).c_str());  // read number of cols
        getline(myfile, line);               // skip "map"
    }
    else  // my benchmark
    {
        char_separator<char> sep(",");
        tokenizer<char_separator<char>> tok(line, sep);
        beg = tok.begin();
        num_of_rows = atoi((*beg).c_str());  // read number of rows
        beg++;
        num_of_cols = atoi((*beg).c_str());  // read number of cols
    }
    assert(num_of_layers > 0);
    map_size = num_of_cols * num_of_rows * num_of_layers;
    my_map.resize(map_size, false);
    // read map (and start/goal locations)
    for (int i = 0; i < num_of_rows; i++)
    {
        getline(myfile, line);
        for (int j = 0; j < num_of_cols; j++)
        {
            for (int k = 0; k < num_of_layers; k++)
            {
                my_map[linearizeCoordinate(i, j, k)] = (line[j] != '.');
            }
        }
    }
    myfile.close();

    // initialize moves_offset array
    /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
    moves_offset[Instance::valid_moves_t::EAST] = 1;
    moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
    moves_offset[Instance::valid_moves_t::WEST] = -1;*/
    return true;
}

void Instance::printMap() const
{
    // While printing/saving, we print the map layer by layer for better
    // visualization.
    // But note that in the implementation the "layer" dimension (aka the
    // height) of the map is the 3rd dimension.
    for (int k = 0; k < num_of_layers; k++)
    {
        for (int i = 0; i < num_of_rows; i++)
        {
            for (int j = 0; j < num_of_cols; j++)
            {
                if (this->my_map[linearizeCoordinate(i, j, k)])
                    cout << '@';
                else
                    cout << '.';
            }
            cout << endl;
        }
        cout << endl;
        cout << endl;
    }
}

void Instance::saveMap() const
{
    ofstream myfile;
    myfile.open(map_fname);
    if (!myfile.is_open())
    {
        cout << "Fail to save the map to " << map_fname << endl;
        return;
    }
    myfile << num_of_rows << "," << num_of_cols << endl;

    // While printing/saving, we print the map layer by layer for better
    // visualization.
    // But note that in the implementation the "layer" dimension (aka the
    // height) of the map is the 3rd dimension.
    for (int k = 0; k < num_of_layers; k++)
    {
        for (int i = 0; i < num_of_rows; i++)
        {
            for (int j = 0; j < num_of_cols; j++)
            {
                if (my_map[linearizeCoordinate(i, j, k)])
                    myfile << "@";
                else
                    myfile << ".";
            }
            myfile << endl;
        }
        myfile << endl;
        myfile << endl;
    }
    myfile.close();
}

bool Instance::loadAgents()
{
    using namespace std;
    using namespace boost;

    string line;
    ifstream myfile(agent_fname.c_str());
    if (!myfile.is_open())
        return false;

    getline(myfile, line);
    if (line[0] == 'v')  // Nathan's benchmark
    {
        if (num_of_agents == 0)
        {
            cerr << "The number of agents should be larger than 0" << endl;
            exit(-1);
        }
        start_locations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);
        char_separator<char> sep("\t");
        for (int i = 0; i < num_of_agents; i++)
        {
            getline(myfile, line);
            tokenizer<char_separator<char>> tok(line, sep);
            tokenizer<char_separator<char>>::iterator beg = tok.begin();
            beg++;  // skip the first number
            beg++;  // skip the map name
            beg++;  // skip the columns
            beg++;  // skip the rows
                    // read start [row,col] for agent i
            int col = atoi((*beg).c_str());
            beg++;
            int row = atoi((*beg).c_str());

            // For third dimension, always start from and end at 0 (ground)
            start_locations[i] = linearizeCoordinate(row, col, 0);
            // read goal [row,col] for agent i
            beg++;
            col = atoi((*beg).c_str());
            beg++;
            row = atoi((*beg).c_str());
            goal_locations[i] = linearizeCoordinate(row, col, 0);
        }
    }
    else  // My benchmark
    {
        char_separator<char> sep(",");
        tokenizer<char_separator<char>> tok(line, sep);
        tokenizer<char_separator<char>>::iterator beg = tok.begin();
        int stored_num_of_agents = atoi((*beg).c_str());
        assert (stored_num_of_agents >= num_of_agents);
        start_locations.resize(num_of_agents);
        goal_locations.resize(num_of_agents);
        for (int i = 0; i < num_of_agents; i++)
        {
            getline(myfile, line);
            tokenizer<char_separator<char>> col_tok(line, sep);
            tokenizer<char_separator<char>>::iterator c_beg = col_tok.begin();
            pair<int, int> curr_pair;
            // read start [row,col] for agent i
            int row = atoi((*c_beg).c_str());
            c_beg++;
            int col = atoi((*c_beg).c_str());
            c_beg++;
            int layer = atoi((*c_beg).c_str());
            start_locations[i] = linearizeCoordinate(row, col, layer);
            // read goal [row,col] for agent i
            c_beg++;
            row = atoi((*c_beg).c_str());
            c_beg++;
            col = atoi((*c_beg).c_str());
            c_beg++;
            layer = atoi((*c_beg).c_str());
            goal_locations[i] = linearizeCoordinate(row, col, layer);
        }
    }
    myfile.close();
    return true;
}

void Instance::printAgents() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i])
             << "," << getColCoordinate(start_locations[i]) << ","
             << getLayerCoordinate(start_locations[i]) << ") ; G=("
             << getRowCoordinate(goal_locations[i]) << ","
             << getColCoordinate(goal_locations[i]) << ","
             << getLayerCoordinate(goal_locations[i]) << ")" << endl;
    }
}

void Instance::saveAgents() const
{
    ofstream myfile;
    myfile.open(agent_fname);
    if (!myfile.is_open())
    {
        cout << "Fail to save the agents to " << agent_fname << endl;
        return;
    }
    myfile << num_of_agents << endl;
    for (int i = 0; i < num_of_agents; i++)
        myfile << getRowCoordinate(start_locations[i]) << ","
               << getColCoordinate(start_locations[i]) << ","
               << getLayerCoordinate(start_locations[i]) << ","
               << getRowCoordinate(goal_locations[i]) << ","
               << getColCoordinate(goal_locations[i]) << ","
               << getLayerCoordinate(goal_locations[i]) << "," << endl;
    myfile.close();
}

list<int> Instance::getNeighbors(int curr) const
{
    list<int> neighbors;

    if (num_of_layers == 1)
    {
        int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols,
                             curr - num_of_cols};
        for (int next : candidates)
        {
            if (validMove(curr, next))
                neighbors.emplace_back(next);
        }
        return neighbors;
    }

    int candidates[6] = {curr + 1,
                         curr - 1,
                         curr + num_of_layers,
                         curr - num_of_layers,
                         curr + num_of_cols * num_of_layers,
                         curr - num_of_cols * num_of_layers};
    for (int next : candidates)
    {
        if (validMove(curr, next))
            neighbors.emplace_back(next);
    }
    return neighbors;
}

// In how many directions the agent can move?
int Instance::getDegree(int curr) const
{
    assert(curr >= 0 && curr < map_size && !my_map[curr]);
    int degree = 0;
    int candidates[6] = {curr + 1,
                         curr - 1,
                         curr + num_of_layers,
                         curr - num_of_layers,
                         curr + num_of_cols * num_of_layers,
                         curr - num_of_cols * num_of_layers};
    for (int next : candidates)
    {
        if (validMove(curr, next))
            degree++;
    }
    return degree;
}

const vector<int>* Instance::getDistances(int root_location, int start_location,
                                          int goal_location)
{
    auto it = distance_matrix.find(root_location);
    if (it != distance_matrix.end())
        return &(it->second);

    struct Node
    {
        int location;
        int value;

        Node() = default;
        Node(int location, int value) : location(location), value(value) {}
        // the following is used to comapre nodes in the OPEN list
        struct compare_node
        {
            // returns true if n1 > n2 (note -- this gives us *min*-heap).
            bool operator()(const Node& n1, const Node& n2) const
            {
                return n1.value >= n2.value;
            }
        };  // used by OPEN (heap) to compare nodes (top of the heap has min
            // f-val, and then highest g-val)
    };

    // Plus 1 to include dummy start location
    vector<int> rst(map_size + 1, MAX_TIMESTEP);

    // generate a heap that can save nodes (and a open_handle)
    boost::heap::pairing_heap<Node, boost::heap::compare<Node::compare_node>>
        heap;
    Node root(root_location, 0);
    rst[root_location] = 0;
    heap.push(root);  // add root to heap
    while (!heap.empty())
    {
        Node curr = heap.top();
        heap.pop();
        for (int next_location : getNeighbors(curr.location))
        {
            if (rst[next_location] > curr.value + 1)
            {
                rst[next_location] = curr.value + 1;
                Node next(next_location, curr.value + 1);
                heap.push(next);
            }
        }
    }

    if (root_location == start_location)
        rst[GLOBAL_VAR::dummy_start_loc] = 0;
    else if (root_location == goal_location)
        rst[GLOBAL_VAR::dummy_start_loc] = rst[start_location] + 1;

    distance_matrix[root_location] = rst;
    return &distance_matrix[root_location];
}

void Instance::saveAgentProfile(boost::filesystem::path filename)
{
    json agent_profile = {
        {"map_dimension", vector<int>{num_of_rows, num_of_cols, num_of_layers}},
        {"costs", this->costs},
        {"values", this->values},
        {"start_locations", this->start_locations},
        {"goal_locations", this->goal_locations}};
    write_to_json(agent_profile, filename);
}

vector<tuple<int, int, int>> Instance::convertAgentLocations(
    vector<int> locations) const
{
    vector<tuple<int, int, int>> agent_locations(locations.size());

    for (int i = 0; i < locations.size(); i++)
    {
        agent_locations[i] = getCoordinate(locations[i]);
    }
    return agent_locations;
}

map<int, int> Instance::initPartialInstance(set<int> global_agents)
{
    int n_local_agents = global_agents.size();
    vector<int> new_start_locations(n_local_agents, -1);
    vector<int> new_goal_locations(n_local_agents, -1);
    vector<double> new_costs(n_local_agents, -1);
    vector<double> new_values(n_local_agents, -1);
    int local_a_id = 0;
    map<int, int> id_map;

    for (int a : global_agents)
    {
        id_map[a] = local_a_id;
        new_start_locations[local_a_id] = this->start_locations[a];
        new_goal_locations[local_a_id] = this->goal_locations[a];
        new_costs[local_a_id] = this->costs[a];
        new_values[local_a_id] = this->values[a];
        local_a_id += 1;
    }

    this->start_locations = new_start_locations;
    this->goal_locations = new_goal_locations;
    this->costs = new_costs;
    this->values = new_values;
    this->num_of_agents = global_agents.size();
    return id_map;
}