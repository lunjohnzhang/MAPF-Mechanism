﻿#pragma once
#include "ConstraintTable.h"
#include "Instance.h"

class LLNode  // low-level node
{
public:
    int location;
    int g_val;
    int h_val = 0;
    LLNode* parent;
    int timestep = 0;
    int num_of_conflicts = 0;
    bool in_openlist = false;
    bool wait_at_goal;  // the action is to wait at the goal vertex or not. This
                        // is used for >lenghth constraints

    // the following is used to compare nodes in the OPEN list
    struct compare_node
    {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const LLNode* n1, const LLNode* n2) const
        {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
            {
                // if (n1->location == n2->location)
                // {
                //     cout << n1->g_val << " + " << n1->h_val << " vs "
                //          << n2->g_val << " + " << n2->h_val << endl;
                //     cout << "Same location: " << n1->location << " "
                //          << n2->location << endl;
                //     cout << "time step " << n1->timestep << " " << n2->timestep
                //          << endl;
                // }

                assert(n1->location != n2->location);

                // Break ties determinstically by location id
                return n1->location > n2->location;
                // if (n1->h_val == n2->h_val)
                // {
                //     // Break ties by location id

                // }
                // // break ties towards smaller h_vals (closer
                // // to goal location)
                // return n1->h_val >= n2->h_val;
            }
            return n1->g_val + n1->h_val > n2->g_val + n2->h_val;
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val,
        // and then highest g-val)

    // the following is used to compare nodes in the FOCAL list
    struct secondary_compare_node
    {
        bool operator()(const LLNode* n1,
                        const LLNode* n2) const  // returns true if n1 > n2
        {
            if (n1->num_of_conflicts == n2->num_of_conflicts)
            {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                {
                    if (n1->h_val == n2->h_val)
                    {
                        return rand() % 2 == 0;  // break ties randomly
                    }
                    // break ties towards smaller h_vals
                    // (closer to goal location)
                    return n1->h_val >= n2->h_val;
                }
                // break ties towards smaller f_vals
                // (prefer shorter solutions)
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
            }
            // n1 > n2 if it has more conflicts
            return n1->num_of_conflicts >= n2->num_of_conflicts;
        }
    };  // used by FOCAL (heap) to compare nodes (top of the heap has min
        // number-of-conflicts)

    LLNode()
        : location(0),
          g_val(0),
          h_val(0),
          parent(nullptr),
          timestep(0),
          num_of_conflicts(0),
          in_openlist(false)
    {
    }

    LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep,
           int num_of_conflicts = 0, bool in_openlist = false)
        : location(location),
          g_val(g_val),
          h_val(h_val),
          parent(parent),
          timestep(timestep),
          num_of_conflicts(num_of_conflicts),
          in_openlist(in_openlist)
    {
    }

    inline int getFVal() const { return g_val + h_val; }
    void copy(const LLNode& other)
    {
        location = other.location;
        g_val = other.g_val;
        h_val = other.h_val;
        parent = other.parent;
        timestep = other.timestep;
        num_of_conflicts = other.num_of_conflicts;
    }
};

class SingleAgentSolver
{
public:
    uint64_t num_expanded = 0;
    uint64_t num_generated = 0;

    double runtime_build_CT = 0;  // runtime of building constraint table
    double runtime_build_CAT =
        0;  // runtime of building conflict avoidance table

    int start_location;
    int goal_location;
    vector<int>
        my_heuristic;  // this is the precomputed heuristic for this agent
    int compute_heuristic(int from, int to)
        const  // compute admissible heuristic between two locations
    {
        return max(get_DH_heuristic(from, to),
                   instance.getManhattanDistance(from, to));
    }
    const Instance& instance;

    virtual Path findOptimalPath(const HLNode& node,
                                 const ConstraintTable& initial_constraints,
                                 const vector<Path*>& paths, int agent,
                                 int lower_bound, bool dummy_start_node) = 0;
    virtual pair<Path, int> findSuboptimalPath(
        const HLNode& node, const ConstraintTable& initial_constraints,
        const vector<Path*>& paths, int agent, int lowerbound, double w,
        double agent_w,
        bool dummy_start_node) = 0;  // return the path and the lowerbound
    virtual int getTravelTime(int start, int end,
                              const ConstraintTable& constraint_table,
                              int upper_bound) = 0;
    virtual string getName() const = 0;

    list<int> getNextLocations(
        int curr) const;  // including itself and its neighbors
    list<int> getNeighbors(int curr) const
    {
        return instance.getNeighbors(curr);
    }

    // int getStartLocation() const {return instance.start_locations[agent]; }
    // int getGoalLocation() const {return instance.goal_locations[agent]; }

    SingleAgentSolver(const Instance& instance, int agent)
        : instance(instance),  // agent(agent),
          start_location(instance.start_locations[agent]),
          goal_location(instance.goal_locations[agent])
    {
        compute_heuristics();
    }

    virtual ~SingleAgentSolver() = default;

protected:
    int min_f_val;  // minimal f value in OPEN
    // int lower_bound; // Threshold for FOCAL
    double w = 1;  // suboptimal bound

    void compute_heuristics();
    int get_DH_heuristic(int from, int to) const
    {
        return abs(my_heuristic[from] - my_heuristic[to]);
    }
};
