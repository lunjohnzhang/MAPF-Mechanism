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
    MDD mdd;

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

    PP(Instance& instance, int screen);
    void preprocess(bool compute_distance_to_start,
                    bool compute_distance_to_goal,
                    bool compute_mdd);  // compute information in each agent
    // return the sum of costs of the solution (INT_MAX if failed to solve)
    std::tuple<int, double, bool, vector<int>> run();
    std::tuple<int, vector<int>> run(int& failed_agent_id, double time_out_sec = 60.0);
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

private:
    // input params
    Instance& instance;
    int screen;
    PathTable path_table;
    SpaceTimeAStar single_agent_planner;

    bool hasSmallerStartGoalDistance(int i, int j) const;
    void quickSort(int low,
                   int high);  // prefer shorter start-goal shortest path
};