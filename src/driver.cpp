/* Copyright (C) Jiaoyang Li
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Confidential
 * Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
 */

/*driver.cpp
 * Solve a MAPF instance on 2D grids.
 */
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "ECBS.h"
#include "PBS.h"
#include "PP.h"

namespace GLOBAL_VAR
{
    int dummy_start_loc;
}

/* Main function */
int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
		("help", "produce help message")

        // Algo
        ("algo", po::value<string>()->required(), "algorithm. one of ['CBS', 'ECBS', 'PP', 'PBS']")

        // params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
        ("agents,a", po::value<string>()->required(), "input file for agents")
        ("saveStats", po::value<bool>()->default_value(true), "Save statistics on disk.")
        ("savePath", po::value<bool>()->default_value(true), "Save path on disk.")
        ("seed", po::value<int>()->default_value(0), "global random seed")
		// ("output,o", po::value<string>(), "output file for statistics")
		// ("outptuPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("cutoffTime,t", po::value<double>()->default_value(7200),
            "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1),
            "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false),
            "write to files some detailed statistics")
        ("nLayers", po::value<int>()->default_value(10),
            "height of the 3D map")
        ("nRuns", po::value<int>()->default_value(1),
            "rapid random nRuns times")
        ("dummyStart", po::value<bool>()->default_value(true),
            "whether to create dummy start node")

        // params for exhaustive PBS
        ("exhaustiveSearch", po::value<bool>()->default_value(true),
            "exhaustive search with PBS")

        // params for mechanism design
        ("cost_mode", po::value<string>()->default_value("uniform"))
        ("value_mode", po::value<string>()->default_value("uniform"))

		// params for CBS node selection strategies
		("highLevelSolver", po::value<string>()->default_value("EES"),
            "the high-level solver (A*, A*eps, EES, NEW)")
		("lowLevelSolver", po::value<bool>()->default_value(true),
            "using suboptimal solver in the low level")
		("inadmissibleH", po::value<string>()->default_value("Zero"),
            "inadmissible heuristics (Zero, Global, Path, Local, Conflict)")
        ("agentSuboptimality", po::value<double>()->default_value(-1),
            "suboptimality bound of each agent; -1 will turn it off")
		("suboptimality", po::value<double>()->default_value(1.2),
            "suboptimality bound")

		// params for CBS improvement
		("heuristics", po::value<string>()->default_value("Zero"),
            "admissible heuristics for the high-level search"
            "(Zero, CG,DG, WDG)")
		("prioritizingConflicts", po::value<bool>()->default_value(true),
            "conflict prioirtization. If true, conflictSelection is used as a"
            "tie-breaking rule.")
		("bypass", po::value<bool>()->default_value(true), "Bypass1")
		("disjointSplitting", po::value<bool>()->default_value(false),
            "disjoint splitting")
		("corridorReasoning", po::value<bool>()->default_value(true),
            "corridor reasoning")
		("targetReasoning", po::value<bool>()->default_value(false),
            "target reasoning")
		("sipp", po::value<bool>()->default_value(0),
            "using SIPPS as the low-level solver")
		;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        cout << desc << endl;
        return 1;
    }

    po::notify(vm);
    if (vm["suboptimality"].as<double>() < 1)
    {
        cerr << "Suboptimal bound should be at least 1!" << endl;
        return -1;
    }

    double agent_w = vm["agentSuboptimality"].as<double>();
    if (agent_w < 1 && agent_w != -1.0)
    {
        cerr << "Agent suboptimal bound should be at least 1 or it should be "
                "-1 to be turned off!"
             << endl;
        return -1;
    }

    high_level_solver_type s;
    if (vm["highLevelSolver"].as<string>() == "A*")
        s = high_level_solver_type::ASTAR;
    else if (vm["highLevelSolver"].as<string>() == "A*eps")
        s = high_level_solver_type::ASTAREPS;
    else if (vm["highLevelSolver"].as<string>() == "EES")
        s = high_level_solver_type::EES;
    else if (vm["highLevelSolver"].as<string>() == "NEW")
        s = high_level_solver_type::NEW;
    else
    {
        cout << "WRONG high level solver!" << endl;
        return -1;
    }

    if (s == high_level_solver_type::ASTAR &&
        vm["suboptimality"].as<double>() > 1)
    {
        cerr << "A* cannot perform suboptimal search!" << endl;
        return -1;
    }

    heuristics_type h;
    if (vm["heuristics"].as<string>() == "Zero")
        h = heuristics_type::ZERO;
    else if (vm["heuristics"].as<string>() == "CG")
        h = heuristics_type::CG;
    else if (vm["heuristics"].as<string>() == "DG")
        h = heuristics_type::DG;
    else if (vm["heuristics"].as<string>() == "WDG")
        h = heuristics_type::WDG;
    else
    {
        cout << "WRONG heuristics strategy!" << endl;
        return -1;
    }

    if ((h == heuristics_type::CG || h == heuristics_type::DG) &&
        vm["lowLevelSolver"].as<bool>())
    {
        cerr << "CG or DG heuristics do not work with low level of suboptimal "
                "search!"
             << endl;
        return -1;
    }

    heuristics_type h_hat;  // inadmissible heuristics
    if (s == high_level_solver_type::ASTAR ||
        s == high_level_solver_type::ASTAREPS ||
        vm["inadmissibleH"].as<string>() == "Zero")
        h_hat = heuristics_type::ZERO;
    else if (vm["inadmissibleH"].as<string>() == "Global")
        h_hat = heuristics_type::GLOBAL;
    else if (vm["inadmissibleH"].as<string>() == "Path")
        h_hat = heuristics_type::PATH;
    else if (vm["inadmissibleH"].as<string>() == "Local")
        h_hat = heuristics_type::LOCAL;
    else if (vm["inadmissibleH"].as<string>() == "Conflict")
        h_hat = heuristics_type::CONFLICT;
    else
    {
        cout << "WRONG inadmissible heuristics strategy!" << endl;
        return -1;
    }

    // Create log logdir
    string algo = vm["algo"].as<string>();
    string timestamp = get_curr_time_str();
    boost::filesystem::path logdir(timestamp + "_" + algo);
    logdir = "logs" / logdir;
    boost::filesystem::create_directories(logdir);

    // Write config to logdir
    write_config_to_file(vm, logdir / "config.json");

    conflict_selection conflict = conflict_selection::EARLIEST;
    node_selection n = node_selection::NODE_CONFLICTPAIRS;

    // Global seeding
    int seed = vm["seed"].as<int>();
    srand(seed);

    ///////////////////////////////////////////////////////////////////////////
    // load the instance
    Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(), seed,
                      vm["agentNum"].as<int>(), 0, 0, vm["nLayers"].as<int>());
    instance.saveAgentProfile(logdir / "agent_profile.json");

    GLOBAL_VAR::dummy_start_loc = instance.map_size;

    int runs = vm["nRuns"].as<int>();
    //////////////////////////////////////////////////////////////////////
    // initialize the solver
    if (vm["lowLevelSolver"].as<bool>() && algo == "ECBS")
    {
        ECBS ecbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        ecbs.setBypass(vm["bypass"].as<bool>());
        ecbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        ecbs.setHeuristicType(h, h_hat);
        ecbs.setTargetReasoning(false);
        ecbs.setMutexReasoning(false);
        ecbs.setConflictSelectionRule(conflict);
        ecbs.setNodeSelectionRule(n);
        ecbs.setSavingStats(vm["stats"].as<bool>());
        ecbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
        ecbs.setLowLevelSolver(agent_w, vm["dummyStart"].as<bool>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            ecbs.clear();
            ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += ecbs.runtime;
            if (ecbs.solution_found)
                break;
            lowerbound = ecbs.getLowerBound();
            ecbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        ecbs.runtime = runtime;
        if (vm["saveStats"].as<bool>())
            ecbs.saveResults((logdir / "stats.csv").string(),
                             vm["agents"].as<string>());
        if (ecbs.solution_found && vm["savePath"].as<bool>())
            ecbs.savePaths((logdir / "paths.txt").string());
        /*size_t pos = (logdir / "stats.csv").string().rfind('.');      //
        position of the file extension string output_name = (logdir /
        "stats.csv").string().substr(0, pos);     // get the name without
        extension cbs.saveCT(output_name); // for debug*/
        if (vm["stats"].as<bool>())
            ecbs.saveStats((logdir / "stats.csv").string(),
                           vm["agents"].as<string>());
        ecbs.saveMechResults(logdir / "result.json");
        ecbs.clearSearchEngines();
    }
    else if (algo == "PP")
    {
        PP pp(instance, vm["screen"].as<int>(), seed);
        pp.setLowLevelSolver(vm["dummyStart"].as<bool>());
        pp.run(runs, logdir, vm["savePath"].as<bool>(),
               vm["cutoffTime"].as<double>());
        pp.saveResults(logdir / "result.json");
    }
    else if (algo == "CBS")
    {
        CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        cbs.setBypass(vm["bypass"].as<bool>());
        cbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        cbs.setHeuristicType(h, h_hat);
        cbs.setTargetReasoning(false);
        cbs.setMutexReasoning(false);
        cbs.setConflictSelectionRule(conflict);
        cbs.setNodeSelectionRule(n);
        cbs.setSavingStats(vm["stats"].as<bool>());
        cbs.setHighLevelSolver(s, vm["suboptimality"].as<double>());
        cbs.setLowLevelSolver(-1, vm["dummyStart"].as<bool>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            cbs.clear();
            cbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += cbs.runtime;
            if (cbs.solution_found)
                break;
            lowerbound = cbs.getLowerBound();
            cbs.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        cbs.runtime = runtime;
        if (vm["saveStats"].as<bool>())
            cbs.saveResults((logdir / "stats.csv").string(),
                            vm["agents"].as<string>());
        if (cbs.solution_found && vm["savePath"].as<bool>())
            cbs.savePaths((logdir / "paths.txt").string());
        if (vm["stats"].as<bool>())
            cbs.saveStats((logdir / "stats.csv").string(),
                          vm["agents"].as<string>());
        cbs.saveMechResults(logdir / "result.json");
        cbs.clearSearchEngines();
    }
    else if (algo == "PBS")
    {
        PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        pbs.setSolverParams(vm["dummyStart"].as<bool>(),
                            vm["exhaustiveSearch"].as<bool>());

        // run
        double runtime = 0;
        pbs.solve(vm["cutoffTime"].as<double>());
        if (vm["saveStats"].as<bool>())
            pbs.saveResults((logdir / "stats.csv").string(),
                            vm["agents"].as<string>());
        if (pbs.solution_found && vm["savePath"].as<bool>())
            pbs.savePaths((logdir / "paths.txt").string());
        pbs.saveMechResults(logdir / "result.json");
        /*size_t pos = (logdir / "stats.csv").string().rfind('.');      //
        position of the file extension string output_name = (logdir /
        "stats.csv").string().substr(0, pos);     // get the name without
        extension cbs.saveCT(output_name); // for debug*/
        pbs.clearSearchEngines();
    }

    return 0;
}