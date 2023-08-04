#pragma once

#include "PBS.h"
#include "SingleAgentSolver.h"

// Merged agents
struct MetaAgent
{
    set<int> agents;  // All agents of the current meta agent

    // (Exhaustive) PBS solver that returns the best partial ordering and
    // solution paths
    PBS* pbs_solver;
    bool solved = false;
    // Map from agent global id to local id, necessary for IDPBS because we
    // split agents in groups.
    map<int, int> id_map;

    bool solve(double time_limit)
    {
        this->pbs_solver->solve(time_limit);
        this->solved = this->pbs_solver->solution_found;
        return this->pbs_solver->solution_found;
    }

    Path* getBestPathByGlobalID(int a)
    {
        return this->pbs_solver->getBestPathByGlobalID(a, this->id_map);
    }

    MetaAgent(set<int> agents, PBS* pbs_solver, map<int, int> id_map)
        : agents(agents), pbs_solver(pbs_solver), id_map(id_map)
    {
        this->solved = false;
    }
    ~MetaAgent()
    {
        pbs_solver->clearSearchEngines();
        delete pbs_solver;
    }
};

class IDPBS
{
public:
    IDPBS(const Instance& global_instance, bool sipp, int screen,
          bool dummystart, bool exhaustive_search);
    void solve(double timelimit);
    const Instance& global_instance;
    bool solution_found;

private:
    bool sipp;
    int screen;
    bool dummystart;
    bool exhaustive_search;
    int num_of_agents;
    bool timeout = false;
    double remain_runtime = 0;
    double runtime = 0;
    clock_t start_time;

    vector<MetaAgent*> all_meta_agents;

    // conflict_map[i][j] = true indicates that meta agent i and meta agents j
    // has conflicts. Note: no need to merge if a meta agent does not conflict
    // with any other meta agents.
    vector<vector<bool>> conflict_map;

    MetaAgent* mergeMetaAgents(set<int> meta_agent_ids);

    // Check conflicts between agents a1 and a2 of meta agents ma1 and ma2
    bool hasConflicts(int ma1, int a1, int ma2, int a2);
    // Check conflicts between two meta agents
    bool hasConflictsBetweenMA(int ma1, int ma2);
    // Check conflicts among all meta agents
    bool hasConflictsAmongMA();
    // Return a list of set of agents to be merged
    vector<set<int>> findMergeMetaAgents();

    void DFS(vector<bool>& visited, set<int>& component, int curr);

    void solveMetaAgent(MetaAgent* meta_agent);

};