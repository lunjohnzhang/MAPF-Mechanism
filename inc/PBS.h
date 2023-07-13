#pragma once
#include "PBSNode.h"
#include "SingleAgentSolver.h"

class PBS
{
public:
	/////////////////////////////////////////////////////////////////////////////////////
	// stats
	double runtime = 0;
	double runtime_generate_child = 0; // runtimr of generating child nodes
	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	double runtime_path_finding = 0; // runtime of finding paths for single agents
	double runtime_detect_conflicts = 0;
	double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

	uint64_t num_HL_expanded = 0;
	uint64_t num_HL_generated = 0;
	uint64_t num_LL_expanded = 0;
	uint64_t num_LL_generated = 0;

	PBSNode* dummy_start = nullptr;
    PBSNode* goal_node = nullptr;

    const Instance& instance;

	bool solution_found = false;
	int solution_cost = -2;

	/////////////////////////////////////////////////////////////////////////////////////////
	// set params
	void setConflictSelectionRule(conflict_selection c) { conflict_seletion_rule = c;}
	void setNodeLimit(int n) { node_limit = n; }

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(double time_limit);

	PBS(const Instance& instance, bool sipp, int screen);
	void clearSearchEngines();
	~PBS();

    void setSolverParams(bool dummy_start_node, bool exhaustive_search)
    {
        this->dummy_start_node = dummy_start_node;
        this->exhaustive_search = exhaustive_search;
    }

	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;
	void saveCT(const string &fileName) const; // write the CT to a file
    void savePaths(const string &fileName) const; // write the paths to a file
	void clear(); // used for rapid random  restart
private:
	conflict_selection conflict_seletion_rule;

    stack<PBSNode*> open_list;
	list<PBSNode*> allNodes_table;

    // exhaustive search.
    // With exhaustive search, PBS will:
    // 1. search through all high level nodes and return the solution with
    //    minimal cost.
    // 2. replan all agents with lower priority than the currently planned high
    //    priority agent.
    bool exhaustive_search = false; // Turn on exhaustive search or not.
    // All nodes with collision-free solution.
    list<PBSNode*> all_solution_nodes;
    int n_solutions = 0;

    list<int> ordered_agents;
    vector<vector<bool>> priority_graph; // [i][j] = true indicates that i is lower than j

    string getSolverName() const;

	int screen;
	bool dummy_start_node = false;
	double time_limit;
	int node_limit = MAX_NODES;

	clock_t start;

	int num_of_agents;


	vector<Path*> paths;
	vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' paths and mdd

    bool generateChild(int child_id, PBSNode* parent, int low, int high);

	bool hasConflicts(int a1, int a2) const;
    bool hasConflicts(int a1, const set<int>& agents) const;
	shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
    int getSumOfCosts() const;
	inline void releaseNodes();

	// print and save
	void printResults() const;
	static void printConflicts(const PBSNode &curr);
    void printPriorityGraph() const;

	bool validateSolution() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;

    //generate random permuattion of agent indices
	vector<int> shuffleAgents() const;

    // check the stop condition and return true if it meets
	bool terminate(PBSNode* curr);

    // Check whether time/node has run out.
    bool timeAndNodeOut();

    void getHigherPriorityAgents(const list<int>::reverse_iterator & p1, set<int>& agents);
    void getLowerPriorityAgents(const list<int>::iterator & p1, set<int>& agents);
    bool hasHigherPriority(int low, int high) const; // return true if agent low is lower than agent high

	// node operators
	void pushNode(PBSNode* node);
    void pushNodes(PBSNode* n1, PBSNode* n2);
	PBSNode* selectNode();

		 // high level search
	bool generateRoot();
    bool findPathForSingleAgent(PBSNode& node, const set<int>& higher_agents, int a, Path& new_path);
	void classifyConflicts(PBSNode &parent);
	void update(PBSNode* node);
	void printPaths() const;

    void topologicalSort(list<int>& stack);
    void topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack);
};
