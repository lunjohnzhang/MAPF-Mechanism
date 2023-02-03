#include "PP.h"

PP::PP(Instance& instance, int screen)
    : instance(instance),
      screen(screen),
      path_table(instance.map_size),
      single_agent_planner(instance, 0)
{
    agents.reserve(instance.num_of_agents);
    for (int i = 0; i < instance.num_of_agents; i++)
    {
        agents.emplace_back(i, instance.start_locations[i],
                            instance.goal_locations[i]);
    }
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
    assert(compute_distance_to_goal ||
           !compute_mdd);  // when compute_mdd is true, compute_distance_to_goal
                           // must be true
    clock_t start_time = clock();
    if (compute_distance_to_start)
    {
        for (auto& agent : agents)
        {
            agent.distance_to_start =
                instance.getDistances(agent.start_location);
        }
    }
    if (compute_distance_to_goal)
    {
        for (auto& agent : agents)
        {
            agent.distance_to_goal = instance.getDistances(agent.goal_location);
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

int PP::run(int& failed_agent_id, double time_out_sec)
{
    assert(ordering.size() == agents.size());
    clock_t start_time = clock();
    int sum_of_costs = 0;
    dependency_graph.resize(ordering.size());
    for (int id : ordering)
    {
        path_table.hit_agents.clear();
        agents[id].path = single_agent_planner.findOptimalPath(
            path_table, *agents[id].distance_to_goal, agents[id].start_location,
            time_out_sec);

        if (time_out_sec < (double)(clock() - start_time) / CLOCKS_PER_SEC)
        {
            sum_of_costs = MAX_COST;
            failed_agent_id = id;
            break;
        }
        for (auto& agent : path_table.hit_agents)
        {
            if (agents[id].path.empty() ||
                agent.second <
                    (int)(agents[id].path.size()) -
                        1)  // only consider agents that lead to shorter paths
                dependency_graph[id].insert(agent.first);
        }
        if (agents[id].path.empty())
        {
            sum_of_costs = MAX_COST;
            failed_agent_id = id;
            break;  // failed, id is the failing agent. in its dependency graph,
                    // at least one pair should be reversed
        }
        sum_of_costs += (int)agents[id].path.size() - 1;
        path_table.insertPath(agents[id].id, agents[id].path);
    }
    runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    return sum_of_costs;
}

int PP::run()
{
    int dummy_int = 0;
    return run(dummy_int);
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
    std::random_shuffle(ordering.begin(),
                        ordering.end());  // randomize the ordering
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