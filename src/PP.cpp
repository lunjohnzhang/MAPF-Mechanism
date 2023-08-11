#include "PP.h"

PP::PP(Instance& instance, int screen, int seed)
    : instance(instance),
      screen(screen),
      seed(seed),
      path_table(instance.map_size + 1),  // plus 1 to include dummy start loc
      single_agent_planner(instance, 0)
{
    agents.reserve(instance.num_of_agents);
    for (int i = 0; i < instance.num_of_agents; i++)
    {
        agents.emplace_back(i, instance.start_locations[i],
                            instance.goal_locations[i]);
    }

    this->gen = mt19937(this->seed);

    this->min_sum_of_cost_wo_i.resize(this->agents.size(), MAX_COST);
    this->max_welfare_wo_i.resize(this->agents.size(), INT_MIN);
    this->best_paths.resize(this->agents.size(), nullptr);
}

void PP::reset()
{
    path_table.reset();
    for (int i = 0; i < instance.num_of_agents; i++)
    {
        agents[i].path.clear();
    }
    dependency_graph.clear();
}

void PP::preprocess(bool compute_distance_to_start,
                    bool compute_distance_to_goal,
                    bool compute_mdd)  // compute information in each agent
{
    // when compute_mdd is true, compute_distance_to_goal must be true
    assert(compute_distance_to_goal || !compute_mdd);
    clock_t start_time = clock();
    if (compute_distance_to_start)
    {
        for (auto& agent : agents)
        {
            agent.distance_to_start = instance.getDistances(
                agent.start_location, agent.start_location,
                agent.goal_location);
        }
    }
    if (compute_distance_to_goal)
    {
        for (auto& agent : agents)
        {
            agent.distance_to_goal = instance.getDistances(
                agent.goal_location, agent.start_location, agent.goal_location);
        }
    }
    if (compute_mdd)
    {
        for (auto& agent : agents)
        {
            agent.mdd.buildMDD(instance, *agent.distance_to_goal,
                               agent.start_location);
        }
    }
    runtime_preprocessing = (double)(clock() - start_time) / CLOCKS_PER_SEC;
}

tuple<double, double> PP::run_once(int& failed_agent_id, int run_id,
                                   double time_out_sec)
{
    assert(ordering.size() == agents.size());
    if (screen > 1)
    {
        cout << "Current order: ";
        for (int id : ordering) cout << id << ", ";
        cout << endl;
    }

    clock_t start_time = clock();
    double sum_of_costs = 0;
    double curr_welfare = 0;
    dependency_graph.resize(ordering.size());
    // vector<double> weighted_path_lengths(agents.size());
    int n = 0;
    for (int id : ordering)
    {
        if (screen > 1)
            cout << "Planning " << n << "th agent: " << id << endl;
        n += 1;
        path_table.hit_agents.clear();
        agents[id].path = single_agent_planner.findOptimalPath(
            path_table, *agents[id].distance_to_goal, agents[id].start_location,
            agents[id].goal_location, time_out_sec, dummy_start_node);

        if (time_out_sec < (double)(clock() - start_time) / CLOCKS_PER_SEC)
        {
            sum_of_costs = MAX_COST;
            curr_welfare = INT_MIN;
            failed_agent_id = id;
            break;
        }
        // for (auto& agent : path_table.hit_agents)
        // {
        //     // only consider agents that lead to shorter paths
        //     if (agents[id].path.empty() ||
        //         agent.second < (int)(agents[id].path.size()) - 1)
        //         dependency_graph[id].insert(agent.first);
        // }
        if (agents[id].path.empty())
        {
            sum_of_costs = MAX_COST;
            curr_welfare = INT_MIN;
            failed_agent_id = id;
            break;  // failed, id is the failing agent. in its dependency graph,
                    // at least one pair should be reversed
        }
        double curr_agent_weighted_path_len =
            (double)(agents[id].path.size() - 1) * this->instance.costs[id];
        sum_of_costs += curr_agent_weighted_path_len;
        curr_welfare +=
            max(this->instance.values[id] - curr_agent_weighted_path_len, 0.0);
        path_table.insertPath(agents[id].id, agents[id].path);
        all_weighted_path_lengths[run_id][id] = curr_agent_weighted_path_len;
    }

    // Success: update min sum of cost without agent i
    if (failed_agent_id == -1)
    {
        for (int i = 0; i < agents.size(); i++)
        {
            double curr_weighted_path_len =
                (double)(agents[i].path.size() - 1) * this->instance.costs[i];
            double curr_sum_of_cost_wo_i =
                sum_of_costs - curr_weighted_path_len;
            double curr_welfare_wo_i =
                curr_welfare -
                max(0.0, this->instance.values[i] - curr_weighted_path_len);

            // Better?
            if (curr_welfare_wo_i > this->max_welfare_wo_i[i])
            {
                this->max_welfare_wo_i[i] = curr_welfare_wo_i;
                this->min_sum_of_cost_wo_i[i] = curr_sum_of_cost_wo_i;
            }
        }
    }

    runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    return std::make_tuple(sum_of_costs, curr_welfare);
}

void PP::storeBestPath()
{
    for (int i = 0; i < this->agents.size(); i++)
    {
        if (this->best_paths[i] != nullptr)
            delete this->best_paths[i];
        this->best_paths[i] = new Path(this->agents[i].path);
    }
}

void PP::savePaths(const string& fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < this->agents.size(); i++)
    {
        output << "Agent " << i << ": ";
        for (const auto& t : *best_paths[i])
            output << "(" << this->instance.getRowCoordinate(t.location) << ","
                   << this->instance.getColCoordinate(t.location) << ","
                   << this->instance.getLayerCoordinate(t.location) << ")->";
        output << endl;
    }
    output.close();
}

void PP::run(int n_runs, double time_out_sec)
{
    this->all_weighted_path_lengths.resize(
        n_runs, vector<double>(this->agents.size(), MAX_COST));

    for (int i = 0; i < n_runs; i++)
    {
        this->preprocess(true, true, true);
        this->computeRandomOrdering();
        int failed_agent_id = -1;
        double sum_of_cost, curr_welfare;
        std::tie(sum_of_cost, curr_welfare) = run_once(failed_agent_id, i);

        // Current run is failed
        if (failed_agent_id >= 0)
        {
            cout << "Failed agent: " << failed_agent_id << endl;
            cout << "Run " << i << " failed" << endl;
            failed_runs.emplace_back(i);
        }
        else
        {
            // Compute upper bound of suboptimality
            double sum_dist_to_goal = 0;
            for (int j = 0; j < this->agents.size(); j++)
            {
                auto agent = this->agents[j];
                vector<int> temp = *agent.distance_to_start;
                sum_dist_to_goal +=
                    (double)temp[agent.goal_location] * this->instance.costs[j];
            }
            double suboptimality = sum_of_cost / sum_dist_to_goal;
            // cout << "Suboptimality: " << suboptimality << endl;

            // all_sum_of_cost_wo_i.emplace_back(sum_of_cost_wo_i);
            // all_weighted_path_lengths.emplace_back(weighted_path_lengths);
            avg_suboptimality += suboptimality;
            n_success += 1;
            avg_sum_of_cost += sum_of_cost;
            // if (suboptimality < min_suboptimality)
            //     min_suboptimality = suboptimality;
            // if (sum_of_cost < min_sum_of_cost)
            // {
            //     min_sum_of_cost = sum_of_cost;
            //     min_sum_of_cost_idx = i;
            // }
            // We want the path that corresponds to the maximum welfare.
            if (curr_welfare > max_social_welfare)
            {
                max_social_welfare = curr_welfare;
                min_sum_of_cost = sum_of_cost;
                max_welfare_idx = i;
                min_sum_of_cost_idx = i;
                min_suboptimality = suboptimality;
                storeBestPath();
            }

            if (!validateSolution())
            {
                cout << "Solution invalid!!!" << endl;
                exit(-1);
            }

            total_runtime += this->runtime;

            if (time_out_sec < total_runtime)
            {
                timeout = true;
                cout << "Timeout at run " << i << endl;
                return;
            }

            if (screen > 0)
            {
                cout << "Run " << i << ": Sum of cost: " << sum_of_cost << ", "
                     << "suboptimality: " << suboptimality << ", "
                     << "runtime: " << this->runtime << endl;
            }
        }
        this->reset();
    }
    avg_suboptimality /= n_success;
    avg_sum_of_cost /= n_success;
    cout << "Average suboptimality: " << avg_suboptimality << endl;
    cout << "Average sum of cost: " << avg_sum_of_cost << endl;
    // cout << "Minimum suboptimality: " << min_suboptimality << endl;
    // cout << "Minimum sum of cost idx: " << min_sum_of_cost_idx << endl;
    cout << "Maximum social welfare: " << max_social_welfare << endl;
    cout << "Maximum social welfare idx: " << max_welfare_idx << endl;
    cout << "Sum of cost of max social welfare path: " << min_sum_of_cost
         << endl;
    cout << "Total runtime: " << total_runtime << endl;
}

void PP::computeDefaultOrdering()  // default ordering uses indices of the
                                   // agents
{
    ordering.resize(agents.size());
    for (int i = 0; i < (int)ordering.size(); i++) ordering[i] = i;
}

void PP::computeRandomOrdering()  // generate a random ordering
{
    if (ordering.size() < agents.size())
    {
        computeDefaultOrdering();
    }

    // randomize the ordering
    std::shuffle(ordering.begin(), ordering.end(), this->gen);
}

void PP::computeSHOrdering(
    bool randomRestart,
    double rr_beta)  // prefer shorter start-goal shortest path
{
    if (ordering.size() < agents.size())
    {
        computeDefaultOrdering();
    }
    if (randomRestart)
    {
        // softmax sampling
        // rr_base has to be negative because SH gives high priority to shorter
        // distance
        vector<int> all_start_goal_distances;
        for (int i = 0; i < agents.size(); i++)
        {
            int d = agents[i].distance_to_goal->at(agents[i].start_location);
            all_start_goal_distances.push_back(d);
        }
        double target_sum = 10.0;
        vector<double> softmax_distances =
            softmax_vector<int>(all_start_goal_distances, rr_beta);
        double sum = std::accumulate(softmax_distances.begin(),
                                     softmax_distances.end(), 0.0);
        std::for_each(softmax_distances.begin(), softmax_distances.end(),
                      [&sum, &target_sum](double& d)
                      { d = d * target_sum / sum; });
        ordering = softmax_ordering(ordering, softmax_distances);
    }
    else
    {
        quickSort(0, (int)ordering.size() - 1);
    }

    if (screen > 1)
    {
        cout << "Start-goal distances: ";
        for (auto i : ordering)
            cout << agents[i].distance_to_goal->at(agents[i].start_location)
                 << ",";
        cout << endl;
    }
}

void PP::computeLHOrdering(
    bool randomRestart,
    double rr_beta)  // prefer longer start-goal shortest path
{
    computeSHOrdering(randomRestart, rr_beta);
    std::reverse(ordering.begin(), ordering.end());

    if (screen > 1)
    {
        cout << "Start-goal distances: ";
        for (auto i : ordering)
            cout << agents[i].distance_to_goal->at(agents[i].start_location)
                 << ",";
        cout << endl;
    }
}

bool PP::hasSmallerStartGoalDistance(int i, int j) const
{
    auto d1 = agents[i].distance_to_goal->at(agents[i].start_location);
    auto d2 = agents[j].distance_to_goal->at(agents[j].start_location);
    if (d1 == d2)
    {
        return (rand() % 2);
    }
    else
    {
        return d1 < d2;
    }
}

void PP::quickSort(int low, int high)
{
    if (low >= high)
        return;
    int pivot = ordering[high];  // pivot
    int i = low;                 // Index of smaller element
    for (int j = low; j <= high - 1; j++)
    {
        // If current element is smaller than or equal to pivot
        if (hasSmallerStartGoalDistance(ordering[j], pivot))
        {
            std::swap(ordering[i], ordering[j]);
            i++;  // increment index of smaller element
        }
    }
    std::swap(ordering[i], ordering[high]);

    quickSort(low, i - 1);   // Before i
    quickSort(i + 1, high);  // After i
}

void PP::printOrdering() const
{
    cout << "[";
    for (auto i : ordering) cout << i << ", ";
    cout << "]" << endl;
}

void PP::printDependencyGraph() const
{
    for (int i = 0; i < (int)dependency_graph.size(); i++)
    {
        if (dependency_graph[i].empty())
            continue;
        for (auto j : dependency_graph[i]) cout << j << "->" << i << ", ";
        cout << endl;
    }
}

void PP::saveResults(boost::filesystem::path filename)
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
    // 6. Any MAPF related stats.

    // Calculate payment
    vector<double> payments(this->agents.size());
    vector<double> utilities(this->agents.size());
    for (int i = 0; i < this->agents.size(); i++)
    {
        payments[i] =
            min_sum_of_cost_wo_i[i] -
            (min_sum_of_cost - all_weighted_path_lengths[max_welfare_idx][i]);

        double curr_welfare = this->instance.values[i] -
                              all_weighted_path_lengths[max_welfare_idx][i];

        utilities[i] = curr_welfare - payments[i];

        // If utility is negative, we assign "no path" to the agent and set
        // social welfare of that agent to 0
        // if (utilities[i] >= 0)
        // {
        //     this->social_welfare += max(0.0, curr_welfare);
        // }
    }

    json mechanism_results = {
        // Agent profile
        {"map_dimension",
         vector<int>{this->instance.num_of_rows, this->instance.num_of_cols,
                     this->instance.num_of_layers}},
        {"costs", this->instance.costs},
        {"values", this->instance.values},
        {"start_coordinates",
         this->instance.convertAgentLocations(this->instance.start_locations)},
        {"goal_coordinates",
         this->instance.convertAgentLocations(this->instance.goal_locations)},
        // MAPF stats
        {"total_runtime", total_runtime},
        {"n_success", n_success},
        {"avg_suboptimality", avg_suboptimality},
        {"avg_sum_of_cost", avg_sum_of_cost},
        {"min_suboptimality", min_suboptimality},
        {"solution_cost", min_sum_of_cost},
        {"social_welfare", max_social_welfare},
        {"min_sum_of_cost_idx", min_sum_of_cost_idx},
        {"max_welfare_idx", max_welfare_idx},
        {"min_sum_of_cost_wo_i", min_sum_of_cost_wo_i},
        {"all_weighted_path_lengths", all_weighted_path_lengths},
        {"failed_runs", failed_runs},
        {"timeout", timeout},
        {"runtime", total_runtime},
        // Mechanism stats
        {"payments", payments},
        {"utilities", utilities}};
    write_to_json(mechanism_results, filename);
}

bool PP::validateSolution() const
{
    // Check whether the paths are feasible.

    int num_of_agents = this->agents.size();
    // double soc = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        // soc += (this->agents[a1].path.size() - 1) * this->instance.costs[a1];
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            size_t min_path_length =
                this->agents[a1].path.size() < this->agents[a2].path.size()
                    ? this->agents[a1].path.size()
                    : this->agents[a2].path.size();
            for (size_t timestep = 0; timestep < min_path_length; timestep++)
            {
                int loc1 = this->agents[a1].path.at(timestep).location;
                int loc2 = this->agents[a2].path.at(timestep).location;

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
                         loc1 ==
                             this->agents[a2].path.at(timestep + 1).location &&
                         loc2 ==
                             this->agents[a1].path.at(timestep + 1).location)
                {
                    cout << "Agents " << a1 << " and " << a2 << " collides at ("
                         << loc1 << "-->" << loc2 << ") at timestep "
                         << timestep << endl;
                    return false;
                }
            }

            // Don't need target conflict as the agents will disappear at goal.
            // if (this->agents[a1].path.size() != this->agents[a2].path.size())
            // {
            // 	int a1_ = this->agents[a1].path.size() <
            // this->agents[a2].path.size() ? a1 : a2; 	int a2_ =
            // this->agents[a1].path.size() < this->agents[a2].path.size() ? a2
            // : a1; 	int loc1 = best_paths[a1_]->back().location; 	for
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

    return true;
}