#pragma once
#include "CBS.h"
#include "ECBSNode.h"

class ECBS : public CBS
{
public:
    ECBS(const Instance& instance, bool sipp, int screen)
        : CBS(instance, sipp, screen)
    {
    }

    // ECBSNode* dummy_start = nullptr;
    // ECBSNode* goal_node = nullptr;

    ////////////////////////////////////////////////////////////////////////////////////////////
    // Runs the algorithm until the problem is solved or time is exhausted
    bool solve(double time_limit, int cost_lowerbound = MIN_COST);
    void clear();  // used for rapid random  restart

private:
    // lower bound of the negated welfare (always negative)
    double welfare_lowerbound = 0;

    // Add an additional subopt for path cost.
    // Path cost are minimized, therefore this subopt >= 1
    double suboptimality_cost = 1.0;

    vector<int> min_f_vals;  // lower bounds of the cost of the shortest path
    vector<pair<Path, int> >
        paths_found_initially;  // contain initial paths found

    pairing_heap<ECBSNode*, compare<ECBSNode::compare_node_by_f> >
        cleanup_list;  // it is called open list in ECBS
    pairing_heap<ECBSNode*, compare<ECBSNode::compare_node_by_inadmissible_f> >
        open_list;  // this is used for EES
    pairing_heap<ECBSNode*, compare<ECBSNode::compare_node_by_d> >
        focal_list;  // this is ued for both ECBS and EES

    void adoptBypass(ECBSNode* curr, ECBSNode* child,
                     const vector<int>& fmin_copy);

    // node operators
    void pushNode(ECBSNode* node);
    ECBSNode* selectNode();
    bool reinsertNode(ECBSNode* node);

    // high level search
    bool generateChild(ECBSNode* child, ECBSNode* curr);
    bool generateRoot();
    bool findPathForSingleAgent(ECBSNode* node, int ag);
    void classifyConflicts(ECBSNode& node);
    void computeConflictPriority(shared_ptr<Conflict>& con, ECBSNode& node);

    // update information
    void updatePaths(ECBSNode* curr);
    void printPaths() const;
};