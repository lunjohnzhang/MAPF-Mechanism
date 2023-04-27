#include <climits>
#include <iostream>
#include <vector>

using namespace std;

vector<int> calculate_payment(int min_sum_of_cost, int min_sum_of_cost_idx,
                              vector<int> all_sum_of_costs,
                              vector<vector<int>> all_path_lengths)
{
    int n_agents = all_path_lengths[0].size();
    int n_runs = all_path_lengths.size();
    vector<int> payments;

    // For each agent i, get the min sum of cost if not considering agent i
    for (int i = 0; i < n_agents; i++)
    {
        // Calculate sum of cost excluding agent i's solution
        vector<int> sum_of_cost_wo_i(n_runs);
        for (int j = 0; j < n_runs; j++)
        {
            sum_of_cost_wo_i[j] = all_sum_of_costs[j] - all_path_lengths[j][i];
        }

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

        payments.emplace_back(
            curr_min_sum_of_cost_wo_i -
            (min_sum_of_cost - all_path_lengths[min_sum_of_cost_idx][i]));
    }

    // Calculate payment of each agent
    return payments;
}

int main(int argc, char** argv)
{
    int min_sum_of_cost = 407;
    int min_sum_of_cost_idx = 10;

    vector<int> all_sum_of_costs{415, 409, 411, 413, 413, 411,
                                 409, 409, 409, 411, 407};
    vector<vector<int>> all_path_lengths{
        {38, 12, 31, 20, 31, 24, 15, 10, 4,  17,
         22, 23, 12, 48, 23, 38, 18, 7,  14, 8},
        {38, 12, 29, 20, 31, 24, 15, 10, 4,  15,
         22, 23, 12, 48, 23, 38, 18, 7,  12, 8},
        {36, 12, 29, 20, 33, 24, 15, 10, 4,  15,
         22, 23, 12, 48, 23, 38, 18, 7,  14, 8},
        {38, 12, 31, 20, 33, 24, 15, 10, 4,  15,
         22, 23, 12, 48, 23, 38, 18, 7,  12, 8},
        {38, 12, 31, 20, 33, 24, 15, 10, 4,  17,
         22, 23, 10, 48, 23, 38, 18, 7,  12, 8},
        {38, 12, 29, 20, 33, 24, 15, 10, 4,  15,
         22, 23, 10, 48, 23, 38, 18, 7,  14, 8},
        {38, 12, 29, 20, 33, 24, 15, 10, 4,  15,
         22, 23, 10, 48, 23, 38, 18, 7,  12, 8},
        {36, 12, 31, 20, 33, 24, 15, 10, 4,  15,
         22, 23, 10, 48, 23, 38, 18, 7,  12, 8},
        {36, 12, 29, 20, 33, 24, 15, 10, 4,  15,
         22, 23, 10, 48, 23, 38, 18, 7,  14, 8},
        {36, 12, 29, 20, 33, 24, 15, 10, 4,  17,
         22, 23, 12, 48, 23, 38, 18, 7,  12, 8},
        {36, 12, 29, 20, 31, 24, 15, 10, 4,  15,
         22, 23, 12, 48, 23, 38, 18, 7,  12, 8}};
    vector<int> payments = calculate_payment(
        min_sum_of_cost,
        min_sum_of_cost_idx,
        all_sum_of_costs,
        all_path_lengths);
    cout << "Payments: ";
    for (int payment : payments)
    {
        cout << payment << " ";
    }
    cout << endl;
}