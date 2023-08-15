#pragma once
#include "CBSHeuristic.h"
#include "CorridorReasoning.h"
#include "MutexReasoning.h"
#include "RectangleReasoning.h"

enum high_level_solver_type
{
    ASTAR,
    ASTAREPS,
    NEW,
    EES
};

class CBS
{
public:
    bool randomRoot =
        false;  // randomize the order of the agents in the root CT node

    /////////////////////////////////////////////////////////////////////////////////////
    // stats
    // total runtime of n runs, used for payment calculation
    double total_runtime = 0;
    // runtime of one run
    double runtime = 0;
    // runtime of generating child nodes
    double runtime_generate_child = 0;
    // runtime of building constraint table
    double runtime_build_CT = 0;
    // runtime of building conflict avoidance table
    double runtime_build_CAT = 0;
    // runtime of finding paths for single agents
    double runtime_path_finding = 0;
    // runtime of detecting conflicts
    double runtime_detect_conflicts = 0;
    // runtime of building heuristic table for the low level
    double runtime_preprocessing = 0;
    // Runtime of calculating payment
    double runtime_calculate_payment = 0;

    uint64_t num_cardinal_conflicts = 0;
    uint64_t num_corridor_conflicts = 0;
    uint64_t num_rectangle_conflicts = 0;
    uint64_t num_target_conflicts = 0;
    uint64_t num_mutex_conflicts = 0;
    uint64_t num_standard_conflicts = 0;

    uint64_t num_adopt_bypass = 0;  // number of times when adopting bypasses

    uint64_t num_HL_expanded = 0;
    uint64_t num_HL_generated = 0;
    uint64_t num_LL_expanded = 0;
    uint64_t num_LL_generated = 0;

    // number of expanded nodes chosen from cleanup list
    uint64_t num_cleanup = 0;
    // number of expanded nodes chosen from open list
    uint64_t num_open = 0;
    // number of expanded nodes chosen from focal list
    uint64_t num_focal = 0;
    // CBSNode* dummy_start = nullptr;
    // CBSNode* goal_node = nullptr;
    HLNode* dummy_start = nullptr;
    HLNode* goal_node = nullptr;

    bool solution_found = false;
    double solution_cost = -2;
    double social_welfare;

    /////////////////////////////////////////////////////////////////////////////////////////
    // set params
    void setHeuristicType(heuristics_type h, heuristics_type h_hat)
    {
        heuristic_helper.type = h;
        heuristic_helper.setInadmissibleHeuristics(h_hat);
    }
    void setPrioritizeConflicts(bool p)
    {
        PC = p;
        heuristic_helper.PC = p;
    }
    void setRectangleReasoning(bool r)
    {
        rectangle_reasoning = r;
        heuristic_helper.rectangle_reasoning = r;
    }
    void setCorridorReasoning(bool c)
    {
        corridor_reasoning = c;
        heuristic_helper.corridor_reasoning = c;
    }
    void setTargetReasoning(bool t)
    {
        target_reasoning = t;
        heuristic_helper.target_reasoning = t;
    }
    void setMutexReasoning(bool m)
    {
        mutex_reasoning = m;
        heuristic_helper.mutex_reasoning = m;
    }
    void setDisjointSplitting(bool d)
    {
        disjoint_splitting = d;
        heuristic_helper.disjoint_splitting = d;
    }
    void setBypass(bool b)
    {
        bypass = b;
    }  // 2-agent solver for heuristic calculation does not need bypass
       // strategy.
    void setConflictSelectionRule(conflict_selection c)
    {
        conflict_selection_rule = c;
        heuristic_helper.conflict_seletion_rule = c;
    }
    void setNodeSelectionRule(node_selection n)
    {
        node_selection_rule = n;
        heuristic_helper.node_selection_rule = n;
    }
    void setSavingStats(bool s)
    {
        save_stats = s;
        heuristic_helper.save_stats = s;
    }
    void setHighLevelSolver(high_level_solver_type s, double w)
    {
        solver_type = s;
        suboptimality = w;
    }
    void setNodeLimit(int n) { node_limit = n; }
    void setLowLevelSolver(double w, bool dummy_start_node)
    {
        agentSuboptimality = w;
        this->dummy_start_node = dummy_start_node;
        this->heuristic_helper.setSolverParams(this->dummy_start_node);
        this->mdd_helper.setParams(this->dummy_start_node);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    // Runs the algorithm until the problem is solved or time is exhausted
    bool solve(double time_limit, int cost_lowerbound = MIN_COST,
               int cost_upperbound = MAX_COST);

    int getLowerBound() const { return cost_lowerbound; }

    CBS(const Instance& instance, bool sipp, int screen);
    CBS(vector<SingleAgentSolver*>& search_engines,
        const vector<ConstraintTable>& constraints,
        vector<Path>& paths_found_initially, int screen);
    void clearSearchEngines();
    ~CBS();

    // Save results
    void saveResults(boost::filesystem::path filename,
                     const string& instanceName);
    // void saveStats(const string& fileName, const string& instanceName);
    void saveCT(const string& fileName) const;     // write the CT to a file
    void savePaths(const string& fileName) const;  // write the paths to a file
    // void saveMechResults(boost::filesystem::path filename) const;
    void clear();  // used for rapid random  restart

    int getInitialPathLength(int agent) const
    {
        return (int)paths_found_initially[agent].size() - 1;
    }

protected:
    bool rectangle_reasoning;  // using rectangle reasoning
    bool corridor_reasoning;   // using corridor reasoning
    bool target_reasoning;     // using target reasoning
    bool disjoint_splitting;   // disjoint splitting
    bool mutex_reasoning;      // using mutex reasoning
    bool bypass;               // using Bypass1
    bool PC;                   // prioritize conflicts
    bool save_stats;
    high_level_solver_type solver_type;  // the solver for the high-level search
    conflict_selection conflict_selection_rule;
    node_selection node_selection_rule;

    MDDTable mdd_helper;
    RectangleReasoning rectangle_helper;
    CorridorReasoning corridor_helper;
    MutexReasoning mutex_helper;
    CBSHeuristic heuristic_helper;

    list<HLNode*> allNodes_table;  // this is ued for both ECBS and EES

    string getSolverName() const;

    int screen;

    double time_limit;
    double suboptimality = 1.0;
    double agentSuboptimality = 1.0;
    double cost_lowerbound = 0;
    double inadmissible_cost_lowerbound;
    int node_limit = MAX_NODES_CBS;
    int cost_upperbound = MAX_COST;
    bool dummy_start_node = false;
    bool timeout = false;
    bool nodeout = false;

    vector<ConstraintTable> initial_constraints;
    clock_t start;

    int num_of_agents;

    vector<Path*> paths;
    vector<Path> paths_found_initially;  // contain initial paths found
    // vector<MDD*> mdds_initially;  // contain initial paths found

    vector<double> payments;
    vector<double> utilities;
    bool payment_calculate_success = true;

    // used to find (single) agents' paths and mdd
    vector<SingleAgentSolver*> search_engines;

    void addConstraints(const HLNode* curr, HLNode* child1,
                        HLNode* child2) const;

    // return agents that violate the constraints
    set<int> getInvalidAgents(const list<Constraint>& constraints);
    // conflicts
    void findConflicts(HLNode& curr);
    void findConflicts(HLNode& curr, int a1, int a2);
    shared_ptr<Conflict> chooseConflict(const HLNode& node) const;
    static void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
                              list<shared_ptr<Conflict>>& copy,
                              const list<int>& excluded_agent);
    void removeLowPriorityConflicts(
        list<shared_ptr<Conflict>>& conflicts) const;
    void computeSecondPriorityForConflict(Conflict& conflict,
                                          const HLNode& node);

    inline void releaseNodes();

    // print and save
    void printResults() const;
    static void printConflicts(const HLNode& curr);

    bool validateSolution() const;
    inline int getAgentLocation(int agent_id, size_t timestep) const;

    // generate random permutation of agent indices
    vector<int> shuffleAgents() const;

    // check the stop condition and return true if it meets
    bool terminate(HLNode* curr);

    // check the conflict is cardinal, semi-cardinal or non-cardinal
    void computeConflictPriority(shared_ptr<Conflict>& con, CBSNode& node);

    double computeWelfare(Path& path, int ag);

private:  // CBS only, cannot be used by ECBS
    // it is called open list in ECBS
    pairing_heap<CBSNode*, compare<CBSNode::compare_node_by_f>> cleanup_list;
    // this is used for EES
    pairing_heap<CBSNode*, compare<CBSNode::compare_node_by_inadmissible_f>>
        open_list;
    // this is ued for both ECBS and EES
    pairing_heap<CBSNode*, compare<CBSNode::compare_node_by_d>> focal_list;

    // node operators
    inline void pushNode(CBSNode* node);
    CBSNode* selectNode();
    inline bool reinsertNode(CBSNode* node);

    // high level search
    bool generateChild(CBSNode* child, CBSNode* curr);
    bool generateRoot();
    bool findPathForSingleAgent(CBSNode* node, int ag, int lower_bound = 0);
    void classifyConflicts(CBSNode& parent);
    // update information
    inline void updatePaths(CBSNode* curr);
    void printPaths() const;

    vector<double> solution_costs_wo_i;
    vector<double> social_welfare_wo_i;
    // Compute VCG payment for CBS
    void computeVCGPayment();
};
