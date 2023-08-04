#include "PBS.h"

#include <algorithm>  // std::shuffle
#include <chrono>     // std::chrono::system_clock
#include <random>     // std::default_random_engine

#include "SIPP.h"
#include "SpaceTimeAStar.h"

PBS::PBS(const Instance& instance, bool sipp, int screen)
    : screen(screen),
      instance(instance),
      num_of_agents(instance.getDefaultNumberOfAgents())
{
    clock_t t = clock();

    search_engines.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (sipp)
            search_engines[i] = new SIPP(instance, i);
        else
            search_engines[i] = new SpaceTimeAStar(instance, i);
    }
    runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

    if (screen >= 2)  // print start and goals
    {
        instance.printAgents();
    }

    this->solution_costs_wo_i.resize(num_of_agents, INT_MAX);
}

bool PBS::solve(double _time_limit)
{
    this->time_limit = _time_limit;

    if (screen > 0)  // 1 or 2
    {
        string name = getSolverName();
        name.resize(SOLVER_NAME_LEN, ' ');
        cout << name << ": ";
    }
    // set timer
    start = clock();

    generateRoot();

    while (!open_list.empty())
    {
        auto curr = selectNode();

        if (terminate(curr))
            break;

        // If not terminate, and under exhaustive PBS, and a solution is found,
        // see if time out.
        if (this->exhaustive_search && this->solution_found)
        {
            // Time/node out, consider as failed even if some solutions are
            // found.
            if (timeAndNodeOut())
                break;

            // Not time/node out, continue to the next node.
            continue;
        }

        curr->conflict = chooseConflict(*curr);

        if (screen > 1)
            cout << "	Expand " << *curr << "	on " << *(curr->conflict)
                 << endl;

        assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and
               !hasHigherPriority(curr->conflict->a2, curr->conflict->a1));
        auto t1 = clock();
        vector<Path*> copy(paths);
        generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
        paths = copy;
        generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
        runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
        pushNodes(curr->children[0], curr->children[1]);
        curr->clear();
    }  // end of while loop

    if (this->exhaustive_search && this->solution_found && screen > 0)
    {
        printResults();
        cout << "Found " << this->n_solutions
             << " solutions, min cost: " << solution_cost << endl;
        cout << "N solutions without overlap: " << this->all_sols.size()
             << endl;
        // cout << "N solutions without overlap: " << this->n_solutions -
        // this->n_overlap_solutions << endl; for (auto item :
        // this->all_sol_priority_g)
        // {
        //     string graph_str = item.first;
        //     vector<Path> curr_paths = item.second;
        //     cout << graph_str << endl;
        //     for (int i = 0; i < num_of_agents; i++)
        //     {
        //         cout << "Agent " << i << " ("
        //              << search_engines[i]
        //                     ->my_heuristic[search_engines[i]->start_location]
        //              << " -->" << curr_paths[i].size() - 1 << "): ";
        //         for (const auto& t : curr_paths[i]) cout << t.location <<
        //         "->"; cout << endl;
        //     }
        //     cout << endl;
        // }
    }
    return solution_found;
}

bool PBS::generateChild(int child_id, PBSNode* parent, int low, int high)
{
    assert(child_id == 0 or child_id == 1);
    parent->children[child_id] = new PBSNode(*parent);
    auto node = parent->children[child_id];
    // node->constraint.set(low, high);
    node->constraint =
        Constraint(low, high, -1, -1, constraint_type::PBS_ORDER);
    priority_graph[high][low] = false;
    priority_graph[low][high] = true;
    if (screen > 2)
        printPriorityGraph();
    topologicalSort(ordered_agents);
    if (screen > 2)
    {
        cout << "Ordered agents: ";
        for (int i : ordered_agents) cout << i << ",";
        cout << endl;
    }

    // map agent i to its position in ordered_agents
    vector<int> topological_orders(num_of_agents);
    auto i = num_of_agents - 1;
    for (const auto& a : ordered_agents)
    {
        topological_orders[a] = i;
        i--;
    }

    // <position in ordered_agents, agent id>
    std::priority_queue<pair<int, int>> to_replan;

    // lookup table to quickly determine if an agent needs replanning.
    vector<bool> lookup_table(num_of_agents, false);

    // With regular PBS, we replan agent low and agents with lower priority
    // than low and have conflicts with agent low.
    if (!this->exhaustive_search)
    {
        to_replan.emplace(topological_orders[low], low);
        lookup_table[low] = true;

        // find conflicts where one agent is higher than high and the other
        // agent is lower than low
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, topological_orders[high]);
        assert(*p == high);
        getHigherPriorityAgents(p, higher_agents);
        higher_agents.insert(high);

        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - topological_orders[low]);
        assert(*p2 == low);
        getLowerPriorityAgents(p2, lower_agents);

        for (const auto& conflict : node->conflicts)
        {
            int a1 = conflict->a1;
            int a2 = conflict->a2;
            if (a1 == low or a2 == low)
                continue;
            if (topological_orders[a1] > topological_orders[a2])
            {
                std::swap(a1, a2);
            }
            if (!lookup_table[a1] and
                lower_agents.find(a1) != lower_agents.end() and
                higher_agents.find(a2) != higher_agents.end())
            {
                to_replan.emplace(topological_orders[a1], a1);
                lookup_table[a1] = true;
            }
        }

        while (!to_replan.empty())
        {
            int a, rank;
            tie(rank, a) = to_replan.top();
            to_replan.pop();
            lookup_table[a] = false;
            if (screen > 2)
                cout << "Replan agent " << a << endl;
            // Re-plan path
            set<int> higher_agents;
            auto p = ordered_agents.rbegin();
            std::advance(p, rank);
            assert(*p == a);
            getHigherPriorityAgents(p, higher_agents);
            assert(!higher_agents.empty());
            if (screen > 2)
            {
                cout << "Higher agents: ";
                for (auto i : higher_agents) cout << i << ",";
                cout << endl;
            }

            Path new_path;
            // Prune the child node if no solution is found.
            if (!findPathForSingleAgent(*node, higher_agents, a, new_path))
            {
                delete node;
                parent->children[child_id] = nullptr;
                return false;
            }

            // Delete old conflicts
            for (auto c = node->conflicts.begin(); c != node->conflicts.end();)
            {
                if ((*c)->a1 == a or (*c)->a2 == a)
                    c = node->conflicts.erase(c);
                else
                    ++c;
            }

            // Update conflicts and to_replan
            set<int> lower_agents;
            auto p2 = ordered_agents.begin();
            std::advance(p2, num_of_agents - 1 - rank);
            assert(*p2 == a);
            getLowerPriorityAgents(p2, lower_agents);
            if (screen > 2 and !lower_agents.empty())
            {
                cout << "Lower agents: ";
                for (auto i : lower_agents) cout << i << ",";
                cout << endl;
            }

            // Find new conflicts
            for (auto a2 = 0; a2 < num_of_agents; a2++)
            {
                // already in to_replan or has higher priority
                if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0)
                    continue;
                auto t = clock();
                if (hasConflicts(a, a2))
                {
                    shared_ptr<Conflict> new_conflict(new Conflict());
                    new_conflict->pbsConflict(a, a2);
                    node->conflicts.emplace_back(new_conflict);

                    // Has a collision with a lower priority agent
                    if (lower_agents.count(a2) > 0)
                    {
                        if (screen > 1)
                            cout << "\t" << a2
                                 << " needs to be replanned due to collisions "
                                    "with "
                                 << a << endl;
                        to_replan.emplace(topological_orders[a2], a2);
                        lookup_table[a2] = true;
                    }
                }
                runtime_detect_conflicts +=
                    (double)(clock() - t) / CLOCKS_PER_SEC;
            }
        }
    }

    // With exhausive PBS, we need to replan all agents with lower priority
    // than agent low, because their path might get better after agent low is
    // replanned, even if they have no conflicts with agent low.
    // else
    // {
    //     // Get lower priority agents
    //     set<int> lower_agents;
    //     auto p2 = ordered_agents.begin();
    //     std::advance(p2, num_of_agents - 1 - topological_orders[low]);
    //     assert(*p2 == low);
    //     getLowerPriorityAgents(p2, lower_agents);

    //     for (auto agent : lower_agents)
    //     {
    //         to_replan.emplace(topological_orders[agent], agent);
    //         lookup_table[agent] = true;
    //     }

    //     while (!to_replan.empty())
    //     {
    //         int a, rank;
    //         tie(rank, a) = to_replan.top();
    //         to_replan.pop();
    //         lookup_table[a] = false;
    //         if (screen > 2)
    //             cout << "Replan agent " << a << endl;
    //         // Re-plan path
    //         set<int> higher_agents;
    //         auto p = ordered_agents.rbegin();
    //         std::advance(p, rank);
    //         assert(*p == a);
    //         getHigherPriorityAgents(p, higher_agents);
    //         assert(!higher_agents.empty());
    //         if (screen > 2)
    //         {
    //             cout << "Higher agents: ";
    //             for (auto i : higher_agents) cout << i << ",";
    //             cout << endl;
    //         }

    //         Path new_path;
    //         // Prune the child node if no solution is found.
    //         if (!findPathForSingleAgent(*node, higher_agents, a, new_path))
    //         {
    //             // cout << "should never happen" << endl;
    //             delete node;
    //             parent->children[child_id] = nullptr;
    //             return false;
    //         }
    //     }

    //     // Delete old conflicts
    //     node->conflicts.clear();

    //     // Find new conflicts
    //     auto t = clock();
    //     for (auto a = 0; a < num_of_agents; a++)
    //     {
    //         for (auto a2 = a + 1; a2 < num_of_agents; a2++)
    //         {
    //             if (hasConflicts(a, a2))
    //             {
    //                 shared_ptr<Conflict> new_conflict(new Conflict());
    //                 new_conflict->pbsConflict(a, a2);
    //                 node->conflicts.emplace_back(new_conflict);
    //             }
    //         }
    //     }
    //     runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
    // }
    else
    {
        // Update to_be_replanned
        node->to_be_replanned[low] = true;
        node->to_be_replanned[high] = false;

        // Get lower priority agents
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - topological_orders[low]);
        assert(*p2 == low);
        getLowerPriorityAgents(p2, lower_agents);

        // Set all lower priority agents than low also as to_be_replanned
        for (auto agent : lower_agents)
        {
            node->to_be_replanned[agent] = true;
        }

        // Add all to_be_replanned agents to to_replan
        for (int agent = 0; agent < num_of_agents; agent++)
        {
            if (node->to_be_replanned[agent])
            {
                to_replan.emplace(topological_orders[agent], agent);
                lookup_table[agent] = true;
            }
        }

        // Delete old conflicts
        node->conflicts.clear();

        while (!to_replan.empty())
        {
            int a, rank;
            tie(rank, a) = to_replan.top();
            to_replan.pop();
            lookup_table[a] = false;
            if (screen > 2)
                cout << "Replan agent " << a << endl;
            // Re-plan path
            set<int> higher_agents;
            auto p = ordered_agents.rbegin();
            std::advance(p, rank);
            assert(*p == a);
            getHigherPriorityAgents(p, higher_agents);
            // assert(!higher_agents.empty());
            if (screen > 2)
            {
                cout << "Higher agents: ";
                for (auto i : higher_agents) cout << i << ",";
                cout << endl;
            }

            Path new_path;
            // Prune the child node if no solution is found.
            if (!findPathForSingleAgent(*node, higher_agents, a, new_path))
            {
                // cout << "should never happen" << endl;
                delete node;
                parent->children[child_id] = nullptr;
                return false;
            }

            // If there is a conflict between the currently replanned agent with
            // any agents not in to_be_replanned, stop replanning.
            auto t = clock();
            set<int> conflict_agents =
                hasConflictsWithNotTobeReplanned(a, node);
            runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
            if (conflict_agents.size() > 0)
            {
                for (int conflict_a2 : conflict_agents)
                {
                    // cout << "child: Agent " << a << " has conflict with agent "
                    //      << conflict_a2 << endl;
                    shared_ptr<Conflict> new_conflict(new Conflict());
                    new_conflict->pbsConflict(a, conflict_a2);
                    node->conflicts.emplace_back(new_conflict);
                }
                break;
            }

            // Current agent has no conflicts with any agents that are not in
            // to_be_replanned. Therefore add it not to_be_replanned.
            node->to_be_replanned[a] = false;
        }

        // printPaths();

        // Find new conflicts
        // auto t = clock();
        // if (node->conflicts.empty())
        // {
        //     for (auto a = 0; a < num_of_agents; a++)
        //     {
        //         for (auto a2 = a + 1; a2 < num_of_agents; a2++)
        //         {
        //             if (!node->to_be_replanned[a] &&
        //                 !node->to_be_replanned[a2] && hasConflicts(a, a2))
        //             {
        //                 shared_ptr<Conflict> new_conflict(new Conflict());
        //                 new_conflict->pbsConflict(a, a2);
        //                 node->conflicts.emplace_back(new_conflict);
        //                 node->to_be_replanned[a] = true;
        //                 node->to_be_replanned[a2] = true;
        //                 break;
        //             }
        //         }
        //         if (!node->conflicts.empty())
        //             break;
        //     }
        // }
    }

    num_HL_generated++;
    node->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "Generate " << *node << endl;
    return true;
}

bool PBS::findPathForSingleAgent(PBSNode& node, const set<int>& higher_agents,
                                 int a, Path& new_path)
{
    clock_t t = clock();

    // Check if current path exist in cache
    bool exist;
    tie(exist, new_path) = checkPathCache(a, higher_agents);

    // Path does not exist in cache, replan.
    if (!exist)
    {
        // TODO: add runtime check to the low level
        //  Build initial constraints based on higher_agents.
        ConstraintTable initial_constraints(this->instance.num_of_cols,
                                            this->instance.map_size);
        for (int a : higher_agents)
        {
            initial_constraints.insert2CT(*paths[a]);
        }
        runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;

        // new_path = search_engines[a]->findOptimalPath(higher_agents, paths,
        // a);
        new_path = search_engines[a]->findOptimalPath(
            node, initial_constraints, paths, a, 0, dummy_start_node);

        num_LL_expanded += search_engines[a]->num_expanded;
        num_LL_generated += search_engines[a]->num_generated;
        runtime_build_CT += search_engines[a]->runtime_build_CT;
        runtime_build_CAT += search_engines[a]->runtime_build_CAT;
        runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
        if (new_path.empty())
            return false;

        // Add new path to cache
        addPathToCache(a, higher_agents, new_path);
    }

    // if (this->exhaustive_search)
    //     assert(paths[a] != nullptr);
    // else
    //     assert(paths[a] != nullptr and !isSamePath(*paths[a], new_path));
    if (!this->exhaustive_search)
        assert(paths[a] != nullptr and !isSamePath(*paths[a], new_path));
    // else
    // {
    //     if (paths[a] != nullptr)
    //         assert(!isSamePath(*paths[a], new_path));
    // }

    int old_path_length = 0;
    if (paths[a] != nullptr)
        old_path_length = (int)paths[a]->size() - 1;

    node.cost += (int)new_path.size() - 1 - old_path_length;
    if (node.makespan > old_path_length)
    {
        node.makespan = max(node.makespan, new_path.size() - 1);
    }
    else
    {
        node.makespan = 0;
        for (int i = 0; i < num_of_agents; i++)
        {
            if (i == a and new_path.size() - 1 > node.makespan)
                node.makespan = new_path.size() - 1;
            else
            {
                if (paths[i] != nullptr)
                    node.makespan = max(node.makespan, paths[i]->size() - 1);
            }
        }
    }
    node.paths.emplace_back(a, new_path);
    paths[a] = &node.paths.back().second;
    assert(!hasConflicts(a, higher_agents));
    return true;
}

// takes the paths_found_initially and UPDATE all (constrained) paths found for
// agents from curr to start
inline void PBS::update(PBSNode* node)
{
    paths.assign(num_of_agents, nullptr);
    priority_graph.assign(num_of_agents, vector<bool>(num_of_agents, false));
    for (auto curr = node; curr != nullptr; curr = curr->parent)
    {
        for (auto& path : curr->paths)
        {
            if (paths[path.first] == nullptr)
            {
                paths[path.first] = &(path.second);
            }
        }
        if (curr->parent != nullptr)  // non-root node
            priority_graph[get<0>(curr->constraint)][get<1>(curr->constraint)] =
                true;
    }
    assert(getSumOfCosts() == node->cost);
}

bool PBS::hasConflicts(int a1, int a2) const
{
    int min_path_length =
        (int)(paths[a1]->size() < paths[a2]->size() ? paths[a1]->size()
                                                    : paths[a2]->size());

    for (int timestep = 0; timestep < min_path_length; timestep++)
    {
        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;

        // Ignore conflicts at dummy start loc
        if (loc1 == GLOBAL_VAR::dummy_start_loc ||
            loc2 == GLOBAL_VAR::dummy_start_loc)
        {
            continue;
        }
        if (loc1 == loc2 or (timestep < min_path_length - 1 and
                             loc1 == paths[a2]->at(timestep + 1).location and
                             loc2 == paths[a1]
                                         ->at(timestep + 1)
                                         .location))  // vertex or edge conflict
        {
            return true;
        }
    }

    // Don't need target conflict as the agents will disappear at goal.
    // if (paths[a1]->size() != paths[a2]->size())
    // {
    // 	int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
    // 	int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
    // 	int loc1 = paths[a1_]->back().location;
    // 	for (int timestep = min_path_length; timestep < (int)paths[a2_]->size();
    // timestep++)
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
bool PBS::hasConflicts(int a1, const set<int>& agents) const
{
    for (auto a2 : agents)
    {
        if (hasConflicts(a1, a2))
            return true;
    }
    return false;
}

set<int> PBS::hasConflictsWithNotTobeReplanned(int a1, PBSNode* node) const
{
    set<int> conflict_agents;
    for (int a2 = 0; a2 < num_of_agents; a2++)
    {
        // a2 is not in the to_be_replanned set and has conflict with a1
        if (a1 != a2 && !node->to_be_replanned[a2] && hasConflicts(a1, a2))
        {
            conflict_agents.insert(a2);
            node->to_be_replanned[a2] = true;
        }
    }
    return conflict_agents;
}

shared_ptr<Conflict> PBS::chooseConflict(const PBSNode& node) const
{
    if (screen == 3)
        printConflicts(node);
    if (node.conflicts.empty())
        return nullptr;
    return node.conflicts.back();
}
int PBS::getSumOfCosts() const
{
    int cost = 0;
    for (const auto& path : paths)
    {
        if (path != nullptr)
            cost += (int)path->size() - 1;
    }
    return cost;
}
inline void PBS::pushNode(PBSNode* node)
{
    // update handles
    open_list.push(node);
    allNodes_table.push_back(node);
}
void PBS::pushNodes(PBSNode* n1, PBSNode* n2)
{
    if (n1 != nullptr and n2 != nullptr)
    {
        if (n1->cost < n2->cost)
        {
            pushNode(n2);
            pushNode(n1);
        }
        else
        {
            pushNode(n1);
            pushNode(n2);
        }
    }
    else if (n1 != nullptr)
    {
        pushNode(n1);
    }
    else if (n2 != nullptr)
    {
        pushNode(n2);
    }
}

PBSNode* PBS::selectNode()
{
    PBSNode* curr = open_list.top();
    open_list.pop();
    update(curr);
    num_HL_expanded++;
    curr->time_expanded = num_HL_expanded;
    if (screen > 1)
        cout << endl << "Pop " << *curr << endl;
    return curr;
}

void PBS::printPaths() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        if (paths[i] != nullptr)
        {
            cout << "Agent " << i << " ("
                 << search_engines[i]
                        ->my_heuristic[search_engines[i]->start_location]
                 << " -->" << paths[i]->size() - 1 << "): ";
            for (const auto& t : *paths[i]) cout << t.location << "->";
            cout << endl;
        }
    }
}

void PBS::printPriorityGraph() const
{
    cout << "Priority graph:";
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if (priority_graph[a1][a2])
                cout << a1 << "<" << a2 << ",";
        }
    }
    cout << endl;
}

void PBS::printResults() const
{
    // if (this->n_solutions > 1)
    //     cout << string(SOLVER_NAME_LEN + 2, ' ');
    if (solution_cost >= 0)  // solved
        cout << "Succeed,";
    else if (solution_cost == -1)  // time_out
        cout << "Timeout,";
    else if (solution_cost == -2)  // no solution
        cout << "No solutions,";
    else if (solution_cost == -3)  // nodes out
        cout << "Nodesout,";

    cout << solution_cost << "," << runtime << "," << num_HL_expanded << ","
         << num_LL_expanded << ","
         <<  // HL_num_generated << "," << LL_num_generated << "," <<
        dummy_start->cost << "," << endl;
    /*if (solution_cost >= 0) // solved
    {
        cout << "fhat = [";
        auto curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "hhat = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "d = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->distance_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "soc = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() - curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
    }*/
}

// void PBS::saveResults(const string& fileName, const string& instanceName)
// const
// {
//     std::ifstream infile(fileName);
//     bool exist = infile.good();
//     infile.close();
//     if (!exist)
//     {
//         ofstream addHeads(fileName);
//         addHeads << "runtime,#high-level expanded,#high-level "
//                     "generated,#low-level expanded,#low-level generated,"
//                  << "solution cost,root g value,"
//                  << "runtime of detecting conflicts,runtime of building "
//                     "constraint tables,runtime of building CATs,"
//                  << "runtime of path finding,runtime of generating child
//                  nodes,"
//                  << "preprocessing runtime,solver name,instance name" <<
//                  endl;
//         addHeads.close();
//     }
//     ofstream stats(fileName, std::ios::app);
//     stats << runtime << "," << num_HL_expanded << "," << num_HL_generated <<
//     ","
//           << num_LL_expanded << "," << num_LL_generated << "," <<

//         solution_cost << "," << dummy_start->cost << "," <<

//         runtime_detect_conflicts << "," << runtime_build_CT << ","
//           << runtime_build_CAT << "," << runtime_path_finding << ","
//           << runtime_generate_child << "," <<

//         runtime_preprocessing << "," << getSolverName() << "," <<
//         instanceName
//           << endl;
//     stats.close();
// }

void PBS::saveCT(const string& fileName) const  // write the CT to a file
{
    // Write the tree graph in dot language to a file
    {
        std::ofstream output;
        output.open(fileName + ".tree", std::ios::out);
        output << "digraph G {" << endl;
        output << "size = \"5,5\";" << endl;
        output << "center = true;" << endl;
        set<PBSNode*> path_to_goal;
        auto curr = goal_node;
        while (curr != nullptr)
        {
            path_to_goal.insert(curr);
            curr = curr->parent;
        }
        for (const auto& node : allNodes_table)
        {
            output << node->time_generated << " [label=\"g=" << node->cost;
            if (node->time_expanded > 0)  // the node has been expanded
            {
                output << "\n #" << node->time_expanded;
            }
            output << "\"]" << endl;

            if (node == dummy_start)
                continue;
            if (path_to_goal.find(node) == path_to_goal.end())
            {
                output << node->parent->time_generated << " -> "
                       << node->time_generated << endl;
            }
            else
            {
                output << node->parent->time_generated << " -> "
                       << node->time_generated << " [color=red]" << endl;
            }
        }
        output << "}" << endl;
        output.close();
    }

    // Write the stats of the tree to a CSV file
    {
        std::ofstream output;
        output.open(fileName + "-tree.csv", std::ios::out);
        // header
        output << "time generated,g value,h value,h^ value,d value,depth,time "
                  "expanded,chosen from,h computed,"
               << "f of best in cleanup,f^ of best in cleanup,d of best in "
                  "cleanup,"
               << "f of best in open,f^ of best in open,d of best in open,"
               << "f of best in focal,f^ of best in focal,d of best in focal,"
               << "praent,goal node" << endl;
        for (auto& node : allNodes_table)
        {
            output << node->time_generated << "," << node->cost << ","
                   << node->depth << "," << node->time_expanded << ",";
            if (node->parent == nullptr)
                output << "0,";
            else
                output << node->parent->time_generated << ",";
            if (node == goal_node)
                output << "1" << endl;
            else
                output << "0" << endl;
        }
        output.close();
    }
}

void PBS::savePaths(const string& fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << "Agent " << i << ": ";
        for (const auto& t : *best_paths[i])
            output << "("
                   << search_engines[0]->instance.getRowCoordinate(t.location)
                   << ","
                   << search_engines[0]->instance.getColCoordinate(t.location)
                   << ","
                   << search_engines[0]->instance.getLayerCoordinate(t.location)
                   << ")->";
        output << endl;
    }
    output.close();
}

void PBS::printConflicts(const PBSNode& curr)
{
    for (const auto& conflict : curr.conflicts)
    {
        cout << *conflict << endl;
    }
}

string PBS::getSolverName() const
{
    return "PBS with " + search_engines[0]->getName();
}

bool PBS::terminate(PBSNode* curr)
{
    runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    if (curr->conflicts.empty())  // no conflicts
    {
        // if (tobeReplanned(curr))
        //     cout << "Error: some agents need to be replanned" << endl;
        bool terminate_search = false;

        // found a solution
        solution_found = true;
        this->n_solutions += 1;
        // printPriorityGraph();

        // ******************* Debug *******************
        // Overlap solution?
        string curr_priority_graph_str = stringifyPriorityGraph();
        vector<Path> current_sol;
        for (int i = 0; i < num_of_agents; i++)
            current_sol.emplace_back(Path(*paths[i]));
        // // Try to find the value.
        // auto it = this->all_sol_priority_g.find(curr_priority_graph_str);

        // // Check if the priority graph was found before.
        // if (it != this->all_sol_priority_g.end())
        // {
        //     // The graph was found.
        //     std::cout << "Graph " << curr_priority_graph_str
        //               << " was found in before." << std::endl;
        // }
        // else
        // {
        //     this->all_sol_priority_g[curr_priority_graph_str] = current_sol;
        // }

        if (this->all_sols.find(current_sol) != this->all_sols.end())
        {
            // cout << "Overlap solution" << endl;
            this->n_overlap_solutions += 1;
        }
        else
            this->all_sols.insert(current_sol);

        // cout << "Solution " << this->n_solutions << endl;
        // printPaths();
        // cout << endl;
        // ******************* Debug *******************

        // With exhaustive PBS, remember the current best solution and
        // continue until all nodes are closed.
        if (this->exhaustive_search)
        {
            // Get the weighted sum of path length.
            double weighted_sol_cost =
                weighted_path_cost(this->paths, this->instance.costs);

            // Store the best path/solution cost as of the current solution
            if (solution_cost == -2 || solution_cost > weighted_sol_cost)
            {
                solution_cost = weighted_sol_cost;
                goal_node = curr;
                storeBestPath();
            }

            // Store the best solution cost without each agent.
            for (int i = 0; i < this->num_of_agents; i++)
            {
                // Calculate current solution cost without agent i
                double curr_sol_cost_wo_i =
                    weighted_sol_cost -
                    this->instance.costs[i] *
                        (double)(this->paths[i]->size() - 1);

                // Better?
                if (curr_sol_cost_wo_i < this->solution_costs_wo_i[i])
                {
                    this->solution_costs_wo_i[i] = curr_sol_cost_wo_i;
                }
            }

            terminate_search = false;
        }

        // With regular PBS, stop on the first valid solution.
        else
        {
            goal_node = curr;
            solution_cost = goal_node->cost;
            terminate_search = true;
            storeBestPath();
        }

        if (!validateSolution())
        {
            cout << "Solution invalid!!!" << endl;
            printPaths();
            exit(-1);
        }
        return terminate_search;
    }
    solution_found = false;

    // Terminate if time/node runs out.
    if (timeAndNodeOut())
        return true;

    return false;
}

bool PBS::timeAndNodeOut()
{
    if (runtime > time_limit || num_HL_expanded > node_limit)
    {  // time/node out
        if (runtime > time_limit)
            timeout = true;
        if (num_HL_expanded > node_limit)
            nodeout = true;
        solution_cost = -1;
        solution_found = false;
        if (screen > 0)  // 1 or 2
            printResults();
        return true;
    }
    return false;
}

// bool PBS::generateRoot()
// {
//     auto root = new PBSNode();
//     root->cost = 0;
//     paths.reserve(num_of_agents);
//     best_paths.resize(num_of_agents, nullptr);

//     set<int> higher_agents;
//     for (auto i = 0; i < num_of_agents; i++)
//     {
//         // CAT cat(dummy_start->makespan + 1);  // initialized to false
//         // updateReservationTable(cat, i, *dummy_start);
//         //  auto new_path = search_engines[i]->findOptimalPath(higher_agents,
//         //  paths, i); For root node, `initial_constraints` contains no
//         //  constraints
//         ConstraintTable initial_constraints(this->instance.num_of_cols,
//                                             this->instance.map_size);
//         auto new_path = search_engines[i]->findOptimalPath(
//             *root, initial_constraints, paths, i, 0, dummy_start_node);
//         num_LL_expanded += search_engines[i]->num_expanded;
//         num_LL_generated += search_engines[i]->num_generated;
//         if (new_path.empty())
//         {
//             cout << "No path exists for agent " << i << endl;
//             return false;
//         }
//         root->paths.emplace_back(i, new_path);
//         paths.emplace_back(&root->paths.back().second);
//         root->makespan = max(root->makespan, new_path.size() - 1);
//         root->cost += (int)new_path.size() - 1;
//     }
//     auto t = clock();
//     root->depth = 0;
//     for (int a1 = 0; a1 < num_of_agents; a1++)
//     {
//         for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
//         {
//             if (hasConflicts(a1, a2))
//             {
//                 shared_ptr<Conflict> new_conflict(new Conflict());
//                 new_conflict->pbsConflict(a1, a2);
//                 root->conflicts.emplace_back(new_conflict);
//             }
//         }
//     }
//     runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
//     num_HL_generated++;
//     root->time_generated = num_HL_generated;
//     if (screen > 1)
//         cout << "Generate " << *root << endl;
//     pushNode(root);
//     dummy_start = root;
//     if (screen >= 2)  // print start and goals
//     {
//         printPaths();
//     }

//     return true;
// }

bool PBS::generateRoot()
{
    auto root = new PBSNode();
    root->cost = 0;
    paths.resize(num_of_agents, nullptr);
    best_paths.resize(num_of_agents, nullptr);
    root->to_be_replanned.resize(num_of_agents, true);

    set<int> higher_agents;
    for (auto a1 = 0; a1 < num_of_agents; a1++)
    {
        // For root node, `initial_constraints` contains no constraints
        ConstraintTable initial_constraints(this->instance.num_of_cols,
                                            this->instance.map_size);
        auto new_path = search_engines[a1]->findOptimalPath(
            *root, initial_constraints, paths, a1, 0, dummy_start_node);
        num_LL_expanded += search_engines[a1]->num_expanded;
        num_LL_generated += search_engines[a1]->num_generated;
        if (new_path.empty())
        {
            cout << "No path exists for agent " << a1 << endl;
            return false;
        }
        root->paths.emplace_back(a1, new_path);
        paths[a1] = &root->paths.back().second;
        root->makespan = max(root->makespan, new_path.size() - 1);
        root->cost += (int)new_path.size() - 1;

        // Check conflicts of the current planned agents and agents not in the
        // to_be_replanned set. If so, stop planning more agents.
        auto t = clock();
        set<int> conflict_agents = hasConflictsWithNotTobeReplanned(a1, root);
        runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
        if (conflict_agents.size() > 0)
        {
            for (int conflict_a2 : conflict_agents)
            {
                // cout << "root: Agent " << a1 << " has conflict with agent "
                //      << conflict_a2 << endl;
                shared_ptr<Conflict> new_conflict(new Conflict());
                new_conflict->pbsConflict(a1, conflict_a2);
                root->conflicts.emplace_back(new_conflict);
            }
            break;
        }

        // Current agent has no conflicts with any agents that are not in
        // to_be_replanned. Therefore add it not to_be_replanned.
        root->to_be_replanned[a1] = false;
    }
    root->depth = 0;
    // for (int a1 = 0; a1 < num_of_agents; a1++)
    // {
    //     for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
    //     {
    //         if (hasConflicts(a1, a2))
    //         {
    //             shared_ptr<Conflict> new_conflict(new Conflict());
    //             new_conflict->pbsConflict(a1, a2);
    //             root->conflicts.emplace_back(new_conflict);
    //         }
    //     }
    // }
    num_HL_generated++;
    root->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "Generate " << *root << endl;
    pushNode(root);
    dummy_start = root;
    if (screen >= 2)  // print start and goals
    {
        printPaths();
    }

    return true;
}

inline void PBS::releaseNodes()
{
    // TODO:: clear open_list
    for (auto& node : allNodes_table) delete node;
    allNodes_table.clear();
}

/*inline void PBS::releaseOpenListNodes()
{
    while (!open_list.empty())
    {
        PBSNode* curr = open_list.top();
        open_list.pop();
        delete curr;
    }
}*/

PBS::~PBS() { releaseNodes(); }

void PBS::clearSearchEngines()
{
    for (auto s : search_engines) delete s;
    search_engines.clear();
}

bool PBS::validateSolution() const
{
    // Check whether the paths are feasible.
    double soc = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        soc += (best_paths[a1]->size() - 1) * this->instance.costs[a1];
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            size_t min_path_length =
                best_paths[a1]->size() < best_paths[a2]->size()
                    ? best_paths[a1]->size()
                    : best_paths[a2]->size();
            for (size_t timestep = 0; timestep < min_path_length; timestep++)
            {
                int loc1 = best_paths[a1]->at(timestep).location;
                int loc2 = best_paths[a2]->at(timestep).location;
                // Ignore conflicts at dummy start loc
                if (loc1 == GLOBAL_VAR::dummy_start_loc ||
                    loc2 == GLOBAL_VAR::dummy_start_loc)
                {
                    continue;
                }
                if (loc1 == loc2)
                {
                    cout << "Agents " << a1 << " and " << a2 << " collides at "
                         << loc1 << " at timestep " << timestep << endl;
                    return false;
                }
                else if (timestep < min_path_length - 1 &&
                         loc1 == best_paths[a2]->at(timestep + 1).location &&
                         loc2 == best_paths[a1]->at(timestep + 1).location)
                {
                    cout << "Agents " << a1 << " and " << a2 << " collides at ("
                         << loc1 << "-->" << loc2 << ") at timestep "
                         << timestep << endl;
                    return false;
                }
            }

            // Don't need target conflict as the agents will disappear at goal.
            // if (best_paths[a1]->size() != best_paths[a2]->size())
            // {
            // 	int a1_ = best_paths[a1]->size() < best_paths[a2]->size() ? a1 :
            // a2; 	int a2_ = best_paths[a1]->size() < best_paths[a2]->size() ?
            // a2 : a1; 	int loc1 = best_paths[a1_]->back().location; 	for
            // (size_t timestep = min_path_length; timestep <
            // best_paths[a2_]->size(); timestep++)
            // 	{
            // 		int loc2 = best_paths[a2_]->at(timestep).location;
            // 		if (loc1 == loc2)
            // 		{
            // 			cout << "Agents " << a1 << " and " << a2 << " collides
            // at
            // "
            // << loc1 << " at timestep " << timestep << endl; 			return
            // false; // It's at least a semi conflict
            // 		}
            // 	}
            // }
        }
    }
    if (!areDoubleSame(soc, solution_cost))
    {
        cout << "The solution cost is wrong!" << endl;
        return false;
    }
    return true;
}

inline int PBS::getAgentLocation(int agent_id, size_t timestep) const
{
    size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
    return paths[agent_id]->at(t).location;
}

// used for rapid random  restart
void PBS::clear()
{
    releaseNodes();
    paths.clear();
    dummy_start = nullptr;
    goal_node = nullptr;
    solution_found = false;
    solution_cost = -2;
}

void PBS::topologicalSort(list<int>& stack)
{
    stack.clear();
    vector<bool> visited(num_of_agents, false);

    // Call the recursive helper function to store Topological
    // Sort starting from all vertices one by one
    for (int i = 0; i < num_of_agents; i++)
    {
        if (!visited[i])
            topologicalSortUtil(i, visited, stack);
    }
}
void PBS::topologicalSortUtil(int v, vector<bool>& visited, list<int>& stack)
{
    // Mark the current node as visited.
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    assert(!priority_graph.empty());
    for (int i = 0; i < num_of_agents; i++)
    {
        if (priority_graph[v][i] and !visited[i])
            topologicalSortUtil(i, visited, stack);
    }
    // Push current vertex to stack which stores result
    stack.push_back(v);
}
void PBS::getHigherPriorityAgents(const list<int>::reverse_iterator& p1,
                                  set<int>& higher_agents)
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.rend(); ++p2)
    {
        if (priority_graph[*p1][*p2])
        {
            auto ret = higher_agents.insert(*p2);
            if (ret.second)  // insert successfully
            {
                getHigherPriorityAgents(p2, higher_agents);
            }
        }
    }
}
void PBS::getLowerPriorityAgents(const list<int>::iterator& p1,
                                 set<int>& lower_subplans)
{
    for (auto p2 = std::next(p1); p2 != ordered_agents.end(); ++p2)
    {
        if (priority_graph[*p2][*p1])
        {
            auto ret = lower_subplans.insert(*p2);
            if (ret.second)  // insert successfully
            {
                getLowerPriorityAgents(p2, lower_subplans);
            }
        }
    }
}

bool PBS::hasHigherPriority(int low, int high)
    const  // return true if agent low is lower than agent high
{
    std::queue<int> Q;
    vector<bool> visited(num_of_agents, false);
    visited[low] = false;
    Q.push(low);
    while (!Q.empty())
    {
        auto n = Q.front();
        Q.pop();
        if (n == high)
            return true;
        for (int i = 0; i < num_of_agents; i++)
        {
            if (priority_graph[n][i] and !visited[i])
                Q.push(i);
        }
    }
    return false;
}

void PBS::storeBestPath()
{
    for (int i = 0; i < num_of_agents; i++)
    {
        if (this->best_paths[i] != nullptr)
            delete this->best_paths[i];
        this->best_paths[i] = new Path(*this->paths[i]);
    }
}

void PBS::saveResults(boost::filesystem::path filename,
                      const string& instanceName) const
{
    // Calculate (if necessary) and store the following results:
    // 1. Weighted sum of path length by the costs of the agents.
    // 2. Weighted sum of path length if ignoring cost of agent i, for each i.
    // 3. Payment of each agent.
    //    payment[i] = "weighted min sum of path length without agent i" -
    //                 ("weighted min sum of path length" - "weighted length of
    //                 path[i]").
    // 4. Utility of each agent.
    //    utility[i] = value[i] - cost[i] * path_length_i - payment_i
    // 5. Agent profile.

    // Calculate payment and utility.
    vector<double> payments(this->num_of_agents);
    vector<double> utilities(this->num_of_agents);
    if (solution_found)
    {
        for (int i = 0; i < this->num_of_agents; i++)
        {
            payments[i] =
                this->solution_costs_wo_i[i] -
                (this->solution_cost -
                 this->instance.costs[i] * (this->best_paths[i]->size() - 1));

            utilities[i] =
                this->instance.values[i] -
                this->instance.costs[i] * (this->best_paths[i]->size() - 1) -
                payments[i];
        }
    }

    json results = {
        {"map_dimension",
         vector<int>{this->instance.num_of_rows, this->instance.num_of_cols,
                     this->instance.num_of_layers}},
        {"costs", this->instance.costs},
        {"values", this->instance.values},
        {"start_coordinates",
         this->instance.convertAgentLocations(this->instance.start_locations)},
        {"goal_coordinates",
         this->instance.convertAgentLocations(this->instance.goal_locations)},
        {"payments", payments},
        {"utilities", utilities},
        {"timeout", timeout},
        {"nodeout", nodeout},
        {"runtime", runtime},
        {"n_solutions", n_solutions},
        {"solution_cost", solution_cost},
        {"num_HL_expanded", num_HL_expanded},
        {"num_HL_generated", num_HL_generated},
        {"num_LL_expanded", num_LL_expanded},
        {"num_LL_generated", num_LL_generated},
        {"root_g_val", dummy_start->cost},
        {"runtime_detect_conflicts", runtime_detect_conflicts},
        {"runtime_build_CT", runtime_build_CT},
        {"runtime_build_CAT", runtime_build_CAT},
        {"runtime_path_finding", runtime_path_finding},
        {"runtime_generate_child", runtime_generate_child},
        {"runtime_preprocessing", runtime_preprocessing},
        {"solver_name", getSolverName()},
        {"instance_name", instanceName},
        {"path_cache_hit", n_cache_hit},
        {"path_cache_miss", n_cache_miss}};
    write_to_json(results, filename);
}

// bool PBS::consistentWithTotalOrder(PBSNode* curr)
// {
//     vector<int> pp_order{
//         2,   11,  95,  108, 116, 72,  87,  60, 80,  45,  101, 73, 31,  35,
//         122, 24,  93,  114, 3,   113, 83,  16, 124, 92,  68,  78, 107, 26,
//         1,   102, 76,  21,  37,  63,  70,  82, 97,  53,  48,  38, 8,   27,
//         58,  98,  121, 28,  86,  7,   22,  46, 96,  112, 103, 71, 109, 39,
//         18,  64,  119, 69,  88,  118, 42,  32, 50,  90,  62,  79, 59,  106,
//         84,  110, 25,  20,  14,  75,  43,  6,  15,  33,  77,  67, 40,  55,
//         49,  57,  104, 34,  66,  120, 41,  65, 12,  36,  4,   17, 99,  9,
//         117, 19,  111, 0,   10,  13,  123, 29, 105, 89,  44,  56, 5,   85,
//         47,  74,  51,  30,  23,  115, 54,  81, 61,  91,  94,  52, 100};
//     // Consistent with the partial order of current goal node?
//     // printPriorityGraph();
//     bool consistent = true;
//     for (auto curr_tmp = curr; curr_tmp != nullptr; curr_tmp =
//     curr_tmp->parent)
//     {
//         if (curr_tmp->parent != nullptr)  // non-root node
//         {
//             int low = get<0>(curr_tmp->constraint);
//             int high = get<1>(curr_tmp->constraint);
//             // cout << "Checking " << low << " < " << high << endl;
//             int low_idx = find_index(pp_order, low);
//             int high_idx = find_index(pp_order, high);
//             // In pp_order, high priority agents has smaller index
//             if (low_idx < high_idx)
//             {
//                 // cout << "Inconsistent: low: " << low << ", high: " << high
//                 <<
//                 // endl;
//                 consistent = false;
//                 break;
//             }
//         }
//     }
//     return consistent;
// }

tuple<bool, Path> PBS::checkPathCache(int agent_id,
                                      const set<int>& higher_agents)
{
    // Form the key
    auto key = getCacheKey(agent_id, higher_agents);

    // Cache hit?
    if (this->path_cache.find(key) != this->path_cache.end())
    {
        this->n_cache_hit += 1;
        return make_tuple(true, this->path_cache[key]);
    }
    this->n_cache_miss += 1;

    return make_tuple(false, Path());
}

void PBS::addPathToCache(int agent_id, const set<int>& higher_agents,
                         Path& new_path)
{
    // Form the key
    auto key = getCacheKey(agent_id, higher_agents);

    // Add to cache
    this->path_cache[key] = new_path;
}

pair<int, set<pair<int, Path>>> PBS::getCacheKey(int agent_id,
                                                 const set<int>& higher_agents)
{
    // Form the key
    set<pair<int, Path>> higher_agents_path;
    for (int higher_agent : higher_agents)
    {
        higher_agents_path.emplace(
            make_pair(higher_agent, *this->paths[higher_agent]));
    }
    return make_pair(agent_id, higher_agents_path);
}

bool PBS::tobeReplanned(PBSNode* node)
{
    for (auto replan : node->to_be_replanned)
    {
        if (replan)
            return true;
    }
    return false;
}

string PBS::stringifyPriorityGraph() const
{
    string priority_graph_str = "";
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = 0; a2 < num_of_agents; a2++)
        {
            if (priority_graph[a1][a2])
                priority_graph_str +=
                    std::to_string(a1) + "<" + std::to_string(a2) + ",";
        }
    }
    return priority_graph_str;
}

Path* PBS::getBestPathByGlobalID(int agent_global_id,
                                 map<int, int> id_map) const
{
    return this->best_paths[id_map.at(agent_global_id)];
}