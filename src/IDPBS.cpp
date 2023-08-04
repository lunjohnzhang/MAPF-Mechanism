#include "IDPBS.h"

IDPBS::IDPBS(const Instance& global_instance, bool sipp, int screen,
             bool dummystart, bool exhaustive_search)
    : global_instance(global_instance),
      num_of_agents(global_instance.num_of_agents),
      sipp(sipp),
      screen(screen),
      dummystart(dummystart),
      exhaustive_search(exhaustive_search)
{
}

MetaAgent* IDPBS::mergeMetaAgents(set<int> meta_agent_ids)
{
    // Don't need to merge if there's only one meta agent
    if (meta_agent_ids.size() <= 1)
        return this->all_meta_agents[*meta_agent_ids.begin()];

    // Merge agents
    set<int> new_agents;
    for (int ma : meta_agent_ids)
    {
        for (int a : this->all_meta_agents[ma]->agents) new_agents.insert(a);
    }

    // Create new PBS solver
    Instance local_instance(this->global_instance);
    map<int, int> id_map = local_instance.initPartialInstance(new_agents);
    PBS* pbs = new PBS(local_instance, this->sipp, this->screen);
    pbs->setSolverParams(this->dummystart, this->exhaustive_search);

    // delete old meta agents
    for (int ma : meta_agent_ids)
    {
        delete this->all_meta_agents[ma];
        this->all_meta_agents[ma] = nullptr;
    }

    MetaAgent* new_meta_agent = new MetaAgent(new_agents, pbs, id_map);
    solveMetaAgent(new_meta_agent);
    return new_meta_agent;
}

bool IDPBS::hasConflictsAmongMA()
{
    // Reinitialize conflicts_map based on current number of agents
    int n_meta_agents = this->all_meta_agents.size();
    conflict_map.resize(n_meta_agents);
    for (int i = 0; i < n_meta_agents; i++)
        conflict_map[i].resize(n_meta_agents, false);
    bool has_conflicts = false;

    for (int i = 0; i < n_meta_agents; i++)
    {
        for (int j = i + 1; j < n_meta_agents; j++)
        {
            if (hasConflictsBetweenMA(i, j))
            {
                this->conflict_map[i][j] = true;
                has_conflicts = true;
            }
        }
    }
    return has_conflicts;
}

// Function for the DFS
void IDPBS::DFS(vector<bool>& visited, set<int>& component, int curr)
{
    visited[curr] = true;
    component.insert(curr);

    for (int i = 0; i < conflict_map[curr].size(); ++i)
    {
        if (conflict_map[curr][i] && !visited[i])
        {
            DFS(visited, component, i);
        }
    }
}

vector<set<int>> IDPBS::findMergeMetaAgents()
{
    int n_meta_agents = this->all_meta_agents.size();
    vector<bool> visited(n_meta_agents, false);
    vector<set<int>> merge_meta_agents;

    for (int i = 0; i < n_meta_agents; ++i)
    {
        if (!visited[i])
        {
            set<int> component;
            DFS(visited, component, i);
            merge_meta_agents.push_back(component);
        }
    }

    return merge_meta_agents;
}

bool IDPBS::hasConflictsBetweenMA(int ma1, int ma2)
{
    for (int a1 : all_meta_agents[ma1]->agents)
    {
        for (int a2 : all_meta_agents[ma2]->agents)
        {
            if (hasConflicts(ma1, a1, ma2, a2))
                return true;
        }
    }
    return false;
}

bool IDPBS::hasConflicts(int ma1, int a1, int ma2, int a2)
{
    MetaAgent* meta_a1 = this->all_meta_agents[ma1];
    MetaAgent* meta_a2 = this->all_meta_agents[ma2];

    auto a1_path = meta_a1->getBestPathByGlobalID(a1);
    auto a2_path = meta_a2->getBestPathByGlobalID(a2);

    int min_path_length =
        (int)(a1_path->size() < a2_path->size() ? a1_path->size()
                                                : a2_path->size());

    for (int timestep = 0; timestep < min_path_length; timestep++)
    {
        int loc1 = a1_path->at(timestep).location;
        int loc2 = a2_path->at(timestep).location;

        // Ignore conflicts at dummy start loc
        if (loc1 == GLOBAL_VAR::dummy_start_loc ||
            loc2 == GLOBAL_VAR::dummy_start_loc)
        {
            continue;
        }
        if (loc1 == loc2 or (timestep < min_path_length - 1 and
                             loc1 == a2_path->at(timestep + 1).location and
                             loc2 == a1_path->at(timestep + 1)
                                         .location))  // vertex or edge conflict
        {
            return true;
        }
    }

    // Don't need target conflict as the agents will disappear at goal.
    // if (a1_path->size() != a2_path->size())
    // {
    // 	int a1_ = a1_path->size() < a2_path->size() ? a1 : a2;
    // 	int a2_ = a1_path->size() < a2_path->size() ? a2 : a1;
    // 	int loc1 = paths[a1_]->back().location;
    // 	for (int timestep = min_path_length; timestep <
    // (int)paths[a2_]->size(); timestep++)
    // 	{
    // 		int loc2 = paths[a2_]->at(timestep).location;
    // 		if (loc1 == loc2)
    // 		{
    // 			return true; // target conflict
    // 		}
    // 	}
    // }
    return false;  // conflict-free
}

void IDPBS::solve(double timelimit)
{
    this->remain_runtime = timelimit;
    this->start_time = clock();

    // Initialize inital meta agents
    this->all_meta_agents.resize(this->num_of_agents, nullptr);
    for (int i = 0; i < this->num_of_agents; i++)
    {
        set<int> agents{i};

        // Initialize instance and PBS solver
        Instance local_instance(this->global_instance);
        map<int, int> id_map = local_instance.initPartialInstance(agents);
        PBS* pbs = new PBS(local_instance, this->sipp, this->screen);
        pbs->setSolverParams(this->dummystart, this->exhaustive_search);

        // Create meta agent
        MetaAgent* ma = new MetaAgent(agents, pbs, id_map);
        this->all_meta_agents[i] = ma;
        solveMetaAgent(ma);
    }

    this->runtime = (double)(clock() - this->start_time) / CLOCKS_PER_SEC;
    this->remain_runtime -= this->runtime;

    while (hasConflictsAmongMA())
    {
        // If there are conflicts, merge the meta agents with conflicts
        vector<set<int>> merge_meta_agents = findMergeMetaAgents();
        vector<MetaAgent*> new_meta_agents;
        for (auto to_merge : merge_meta_agents)
            new_meta_agents.emplace_back(mergeMetaAgents(to_merge));
        this->all_meta_agents = new_meta_agents;
    }

    double sum_of_cost = 0;
    for (auto ma : this->all_meta_agents)
        sum_of_cost += ma->pbs_solver->solution_cost;
    cout << "sum of cost " << sum_of_cost << endl;
}

void IDPBS::solveMetaAgent(MetaAgent* meta_agent)
{
    if (!meta_agent->solved)
    {
        bool solution_found = meta_agent->solve(this->remain_runtime);
        this->runtime = (double)(clock() - this->start_time) / CLOCKS_PER_SEC;
        this->remain_runtime -= this->runtime;

        // Timeout
        if (this->remain_runtime <= 0)
        {
            timeout = true;
            return;
        }

        // No solution
        if (!solution_found)
        {
            this->solution_found = false;
            return;
        }
    }
}