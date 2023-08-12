#pragma once
#include "MDD.h"
#include "PathTable.h"
#include "SpaceTimeAStar.h"
#include "common.h"

struct Agent
{
    int id;
    int start_location;
    int goal_location;

    // actual distance from start_location to any location on the map
    const vector<int>* distance_to_start = nullptr;

    // actual distance from goal_location to any location on the map
    const vector<int>* distance_to_goal = nullptr;
    // MDD mdd;

    Path path;

    Agent(int id, int start_location, int goal_location)
        : id(id), start_location(start_location), goal_location(goal_location)
    {
    }
};

class PP
{
public:
    vector<Agent> agents;
    double runtime_preprocessing = 0;
    double runtime = 0;
    vector<int> ordering;
    vector<set<int>> dependency_graph;  // entry[i][j] means that agent i has
                                        // lower priority than agent j

    PP(Instance& instance, int screen, int seed);
    void setLowLevelSolver(bool dummy_start_node)
    {
        this->dummy_start_node = dummy_start_node;
    }
    void preprocess(bool compute_distance_to_start,
                    bool compute_distance_to_goal,
                    bool compute_mdd);  // compute information in each agent
    void run(int n_runs, double time_out_sec);
    bool validateSolution() const;

    // return the sum of costs of the solution (MAX_COST if failed to solve)
    tuple<double, double> run_once(int& failed_agent_id, int run_id,
                                   double time_out_sec = 60.0);
    void reset();

    // default ordering uses indices of the agents
    void computeDefaultOrdering();
    void computeRandomOrdering();  // generate a random ordering
    void computeLHOrdering(
        bool randomRestart = false,
        double rr_beta = -0.5);  // prefer longer start-goal shortest path
    void computeSHOrdering(
        bool randomRestart = false,
        double rr_beta = -0.5);  // prefer shorter start-goal shortest path

    void printOrdering() const;
    void printDependencyGraph() const;
    void savePaths(const string& fileName) const;
    void saveResults(boost::filesystem::path filename);
    void storeBestPath();

private:
    // input params
    Instance& instance;
    int screen;
    // PathTable path_table;
    SpaceTimeAStar single_agent_planner;
    vector<Path*> best_paths;

    // Whether use dummy start node at single agent solver
    bool dummy_start_node;

    mt19937 gen;
    int seed;

    bool hasSmallerStartGoalDistance(int i, int j) const;
    void quickSort(int low,
                   int high);  // prefer shorter start-goal shortest path

    // Stats of monte carlo PP
    double avg_suboptimality = 0;
    double avg_sum_of_cost = 0;
    // Suboptimality that corresponds to the maximum welfare
    double min_suboptimality = INT_MAX;
    // Sum of cost corresponds to the maximum welfare
    double min_sum_of_cost = MAX_COST;
    double max_social_welfare = INT_MIN;

    // Idx of the run that gets the min sum of cost
    int min_sum_of_cost_idx = -1;
    int max_welfare_idx = -1;
    int n_success = 0;
    double total_runtime = 0;
    bool timeout = false;

    // [i] stores the weighted sum of cost without agent i that corresponds to
    // the maximum welfare
    vector<double> min_sum_of_cost_wo_i;
    // [i] stores the max welfare without agent i
    vector<double> max_welfare_wo_i;

    // [i][j] stores the weighted path length of the j-th agent
    // in the i-th run
    vector<vector<double>> all_weighted_path_lengths;

    // Failed runs
    vector<int> failed_runs;
};