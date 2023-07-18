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

void write_config_to_file(boost::program_options::variables_map vm,
                          boost::filesystem::path filename)
{
    json config;
    for (const auto& it : vm)
    {
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
            std::cout << "Unknown var type for config param" << it.first
                      << std::endl;
    }
    write_to_json(config, filename);
}

double weighted_path_cost(vector<Path*> paths, vector<double> costs)
{
    assert(paths.size() == costs.size());

    double weighted_sum;
    for (int i = 0; i < paths.size(); i++)
    {
        weighted_sum += (double)(paths[i]->size() - 1) * costs[i];
    }
    return weighted_sum;
}

bool areDoubleSame(double dFirstVal, double dSecondVal)
{
    return std::abs(dFirstVal - dSecondVal) < 1E-5;
}