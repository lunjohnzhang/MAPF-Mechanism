#include "common.h"

std::ostream& operator<<(std::ostream& os, const Path& path)
{
    for (const auto& state : path)
    {
        os << state.location;  // << "(" << state.is_single() << "),";
    }
    return os;
}

bool isSamePath(const Path& p1, const Path& p2)
{
    if (p1.size() != p2.size())
        return false;
    for (unsigned i = 0; i < p1.size(); i++)
    {
        if (p1[i].location != p2[i].location)
            return false;
    }
    return true;
}

string doubleToStr(double d, int precision)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << d;
    std::string s = stream.str();
    return s;
}

vector<int> softmax_ordering(vector<int>& agents_to_arrange,
                             vector<double>& predictions_to_arrange)
{
    assert(agents_to_arrange.size() == predictions_to_arrange.size());  // debug
    int num_agents = agents_to_arrange.size();
    vector<int> ordering;
    double curr_total_sum = std::accumulate(predictions_to_arrange.begin(),
                                            predictions_to_arrange.end(), 0.0);

    while (!agents_to_arrange.empty())
    {
        double running_sum = 0;
        double curr_total_sum = std::accumulate(
            predictions_to_arrange.begin(), predictions_to_arrange.end(), 0.0);
        double cutoff_value = fRand(0, curr_total_sum);
        for (int k = 0; k < predictions_to_arrange.size(); k++)
        {
            running_sum += predictions_to_arrange[k];
            if (running_sum >= cutoff_value)
            {
                ordering.push_back(agents_to_arrange[k]);
                agents_to_arrange.erase(agents_to_arrange.begin() + k);
                predictions_to_arrange.erase(predictions_to_arrange.begin() +
                                             k);
                running_sum = 0;
                break;
            }
        }
    }
    return ordering;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

vector<int> calculate_payment(int min_sum_of_cost,
                              vector<vector<int>> all_sum_of_cost_wo_i)
{
    int n_agents = all_sum_of_cost_wo_i[0].size();
    int n_runs = all_sum_of_cost_wo_i.size();
    vector<int> payments;

    // For each agent i, get the min sum of cost if not considering agent i
    for (int i = 0; i < n_agents; i++)
    {
        int curr_min_sum_of_cost_wo_i = INT_MAX;
        for (int j = 0; j < n_runs; j++)
        {
            if (all_sum_of_cost_wo_i[j][i] < curr_min_sum_of_cost_wo_i)
            {
                curr_min_sum_of_cost_wo_i = all_sum_of_cost_wo_i[j][i];
            }
        }
        payments.emplace_back(min_sum_of_cost - curr_min_sum_of_cost_wo_i);
    }

    // Calculate payment of each agent
    return payments;
}

string get_curr_time_str()
{
    // Get the current time
    auto now = std::chrono::system_clock::now();

    // Convert the time to a time_t object
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    // Convert the time to a string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = ss.str();
    return timestamp;
}