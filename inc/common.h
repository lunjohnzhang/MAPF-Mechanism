#pragma once
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <ctime>
#include <fstream>
#include <iomanip>   // std::setprecision
#include <iostream>  // std::cout, std::fixed
#include <list>
#include <set>
#include <tuple>
#include <vector>
#include <numeric>
#include <chrono>
#include <stack>
#include <nlohmann/json.hpp>

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::pairing_heap;
using std::cerr;
using std::clock;
using std::cout;
using std::endl;
using std::get;
using std::list;
using std::make_pair;
using std::make_shared;
using std::make_tuple;
using std::map;
using std::max;
using std::min;
using std::ofstream;
using std::pair;
using std::set;
using std::shared_ptr;
using std::string;
using std::tie;
using std::tuple;
using std::vector;
using std::get;
using std::stack;
using json = nlohmann::json;

// #define NDEBUG

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2
#define SOLVER_NAME_LEN 35

struct PathEntry
{
    int location = -1;
    // bool single = false;
    // int mdd_width;

    // bool is_single() const {
    //  return mdd_width == 1;
    //}
    PathEntry(int loc = -1) { location = loc; }
};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

string doubleToStr(double d, int precision);

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
/*struct three_tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
        auto h1 = std::hash<T1>{}(get<0>(p));
        auto h2 = std::hash<T2>{}(get<1>(p));
        auto h3 = std::hash<T3>{}(get<2>(p));
        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2 ^ h3;
    }
};*/

double fRand(double fMin, double fMax);

vector<int> softmax_ordering(vector<int>& agents_to_arrange, vector<double>& predictions_to_arrange);

template <typename W>
vector<double> softmax_vector(const vector<W>& to_normalize, double beta)
{
    vector<double> retvec(to_normalize.size(), 0);
    std::transform(to_normalize.begin(), to_normalize.end(), retvec.begin(), [&beta](W d) -> double {
        return std::exp(beta * d);
        });

    double sum_of_expo = std::accumulate(retvec.begin(), retvec.end(), 0.0);
    std::for_each(retvec.begin(), retvec.end(), [&sum_of_expo](double& d) { d = d / sum_of_expo; });

    return retvec;
}

vector<int> calculate_payment(
    int min_sum_of_cost,
    int min_sum_of_cost_idx,
    vector<int> all_sum_of_costs,
    vector<vector<int>> all_path_lengths);

string get_curr_time_str();

void write_to_json(json to_write, boost::filesystem::path filename);

void write_config_to_file(
    boost::program_options::variables_map vm,
    boost::filesystem::path filename);