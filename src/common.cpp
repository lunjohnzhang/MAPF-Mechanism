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

vector<int> calculate_payment(
    int min_sum_of_cost,
    int min_sum_of_cost_idx,
    vector<int> all_sum_of_costs,
    vector<vector<int>> all_path_lengths)
{
    int n_agents = all_path_lengths[0].size();
    int n_runs = all_path_lengths.size();
    vector<int> payments;

    // For each agent i, get the min sum of cost if not considering agent i
    for (int i = 0; i < n_agents; i++)
    {
        // cout << "with i    : ";
        // for (int j = 0; j < n_runs; j++)
        // {
        //     cout << std::setw(4) << all_sum_of_costs[j] << " ";
        // }
        // cout << endl;

        // Calculate sum of cost excluding agent i's solution
        vector<int> sum_of_cost_wo_i(n_runs);
        // cout << "without i : ";
        for (int j = 0; j < n_runs; j++)
        {
            sum_of_cost_wo_i[j] = all_sum_of_costs[j] - all_path_lengths[j][i];
            // cout << std::setw(4) << sum_of_cost_wo_i[j] << " ";
        }
        // cout << endl;

        // cout << "path len i: ";
        // for (int j = 0; j < n_runs; j++)
        // {
        //     cout << std::setw(4) << all_path_lengths[j][i] << " ";
        // }
        // cout << endl << endl;

        // Get the min of the sum of cost excluding i
        int curr_min_sum_of_cost_wo_i = INT_MAX;
        int curr_min_sum_of_cost_wo_i_idx = -1;
        for (int j = 0; j < n_runs; j++)
        {
            if (curr_min_sum_of_cost_wo_i > sum_of_cost_wo_i[j])
            {
                curr_min_sum_of_cost_wo_i = sum_of_cost_wo_i[j];
                curr_min_sum_of_cost_wo_i_idx = j;
            }
        }

        cout << curr_min_sum_of_cost_wo_i_idx << " " << min_sum_of_cost_idx << endl;
        cout << curr_min_sum_of_cost_wo_i << " - "
             << "(" << min_sum_of_cost << " - "
             << all_path_lengths[min_sum_of_cost_idx][i] << ")" << endl;

        // for (int j = 0; j < n_runs; j++)
        // {
        //     if (all_path_lengths[j][i] < curr_min_sum_of_cost_wo_i)
        //     {
        //         curr_min_sum_of_cost_wo_i = all_sum_of_cost_wo_i[j][i];
        //     }
        // }
        payments.emplace_back(
            curr_min_sum_of_cost_wo_i -
            (min_sum_of_cost - all_path_lengths[min_sum_of_cost_idx][i]));
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

void write_to_json(json to_write, boost::filesystem::path filename)
{
    // Open a file for writing
    std::ofstream outfile(filename.string());

    // Write the JSON object to the file
    outfile << std::setw(4) << std::setfill(' ') << to_write << std::endl;

    // Close the file
    outfile.close();
}

void write_config_to_file(
    boost::program_options::variables_map vm,
    boost::filesystem::path filename)
{
    json config;
    for (const auto& it : vm) {
        // std::cout << it.first.c_str() << " ";
        auto& value = it.second.value();
        if (auto v = boost::any_cast<bool>(&value))
            config[it.first] = *v;
        else if (auto v = boost::any_cast<int>(&value))
            config[it.first] = *v;
        else if (auto v = boost::any_cast<double>(&value))
            config[it.first] = *v;
        else if (auto v = boost::any_cast<std::string>(&value))
            config[it.first] = *v;
        else
            std::cout << "Unknown var type for config param"
                      << it.first << std::endl;
    }
    write_to_json(config, filename);
}