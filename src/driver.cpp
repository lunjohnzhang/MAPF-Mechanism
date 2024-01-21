/*driver.cpp
 * Solve a MAPF instance on 2D or 3D grids.
 */
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>

#include "ECBS.h"
#include "IDPBS.h"
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

        // Algo, PP1 refers to PP with nRuns = 1, which is
        // first-come-first-serve
        ("algo", po::value<string>()->required(), "algorithm. one of ['CBS', 'ECBS', 'PP', 'PBS', 'PP1', 'IDPBS']")

        // params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
        ("agents,a", po::value<string>()->required(), "input file for agents")
        // ("saveStats", po::value<bool>()->default_value(true), "Save statistics on disk.")
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
            "number of times to run for Monte Carlo PP")
        ("nRestarts", po::value<int>()->default_value(0),
            "number of times to restart for CBS/ECBS. They runs for nRestarts+1 times")
        ("dummyStart", po::value<bool>()->default_value(true),
            "whether to create dummy start node")
        ("root_logdir", po::value<string>()->default_value("logs"))

        // params for exhaustive PBS
        ("exhaustiveSearch", po::value<bool>()->default_value(true),
            "exhaustive search with PBS")

        // params for mechanism design
        ("cost", po::value<string>()->required())
        ("value", po::value<string>()->required())
        ("cbs_payment", po::value<bool>()->default_value(false),
            "compute VCG payment for CBS or not")

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
        ("rectangleReasoning", po::value<bool>()->default_value(true),
            "rectangle reasoning")
		("corridorReasoning", po::value<bool>()->default_value(true),
            "corridor reasoning")
		("targetReasoning", po::value<bool>()->default_value(false),
            "target reasoning")
		("sipp", po::value<bool>()->default_value(false),
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
    string uuid = get_uuid();
    boost::filesystem::path logdir(
        timestamp + "_" + algo +
        "_k=" + std::to_string(vm["agentNum"].as<int>()) +
        "_seed=" + std::to_string(vm["seed"].as<int>()) + "_" + uuid);
    logdir = vm["root_logdir"].as<string>() / logdir;
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
                      vm["cost"].as<string>(), vm["value"].as<string>(),
                      vm["agentNum"].as<int>(), 0, 0, vm["nLayers"].as<int>());

    GLOBAL_VAR::dummy_start_loc = instance.map_size;

    // Log basic info of current exp
    cout << "Running " << algo << ", map: " << vm["map"].as<string>()
         << ", n_layers: " << vm["nLayers"].as<int>()
         << ", n_agents: " << vm["agentNum"].as<int>() << ", seed: " << seed
         << endl
         << endl;

    // Turn on rectangle reasoning only in 2D maps.
    bool rectangle_reasoning = false;
    if (vm["rectangleReasoning"].as<bool>() && instance.num_of_layers == 1)
        rectangle_reasoning = true;

    //////////////////////////////////////////////////////////////////////
    // initialize the solver
    if (vm["lowLevelSolver"].as<bool>() && algo == "ECBS")
    {
        ECBS ecbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        ecbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        ecbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        ecbs.setBypass(vm["bypass"].as<bool>());
        ecbs.setRectangleReasoning(rectangle_reasoning);
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
        int runs = vm["nRestarts"].as<int>() + 1;
        for (int i = 0; i < runs; i++)
        {
            ecbs.clear();
            ecbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += ecbs.runtime;
            if (ecbs.solution_found)
                break;
            lowerbound = ecbs.getLowerBound();
            ecbs.randomRoot = true;
            cout << "ECBS: Failed to find solutions in Run " << i << endl;
        }
        ecbs.runtime = runtime;
        // if (vm["saveStats"].as<bool>())
        //     ecbs.saveResults((logdir / "stats.csv").string(),
        //                      vm["agents"].as<string>());
        if (ecbs.solution_found && vm["savePath"].as<bool>())
            ecbs.savePaths((logdir / "paths.txt").string());
        /*size_t pos = (logdir / "stats.csv").string().rfind('.');      //
        position of the file extension string output_name = (logdir /
        "stats.csv").string().substr(0, pos);     // get the name without
        extension cbs.saveCT(output_name); // for debug*/
        // if (vm["stats"].as<bool>())
        //     ecbs.saveStats((logdir / "stats.csv").string(),
        //                    vm["agents"].as<string>());
        ecbs.saveResults(logdir / "result.json", vm["agents"].as<string>(),
                         vm["cbs_payment"].as<bool>());
        ecbs.clearSearchEngines();
    }
    else if (algo == "PP" || algo == "PP1")
    {
        int runs = vm["nRuns"].as<int>();
        if (algo == "PP1")
            runs = 1;
        PP pp(instance, vm["screen"].as<int>(), seed);
        pp.setLowLevelSolver(vm["dummyStart"].as<bool>());
        pp.run(runs, vm["cutoffTime"].as<double>());
        pp.saveResults(logdir / "result.json");
        if (vm["savePath"].as<bool>() && pp.solution_found)
            pp.savePaths((logdir / "paths.txt").string());
        pp.clearSearchEngines();
    }
    else if (algo == "CBS")
    {
        CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        cbs.setBypass(vm["bypass"].as<bool>());
        cbs.setRectangleReasoning(rectangle_reasoning);
        cbs.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        cbs.setHeuristicType(h, h_hat);
        cbs.setTargetReasoning(false);
        cbs.setMutexReasoning(false);
        cbs.setConflictSelectionRule(conflict);
        cbs.setNodeSelectionRule(n);
        cbs.setSavingStats(vm["stats"].as<bool>());
        cbs.setHighLevelSolver(s, 1.0);
        cbs.setLowLevelSolver(-1, vm["dummyStart"].as<bool>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = MIN_COST;
        int runs = vm["nRestarts"].as<int>() + 1;
        for (int i = 0; i < runs; i++)
        {
            cbs.clear();
            cbs.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += cbs.runtime;
            if (cbs.solution_found)
                break;
            lowerbound = cbs.getLowerBound();
            cbs.randomRoot = true;
            cout << "CBS: Failed to find solutions in Run " << i << endl;
        }
        cbs.runtime = runtime;
        // if (vm["saveStats"].as<bool>())
        //     cbs.saveResults((logdir / "stats.csv").string(),
        //                     vm["agents"].as<string>());
        if (cbs.solution_found && vm["savePath"].as<bool>())
            cbs.savePaths((logdir / "paths.txt").string());
        // if (vm["stats"].as<bool>())
        //     cbs.saveStats((logdir / "stats.csv").string(),
        //                   vm["agents"].as<string>());
        cbs.saveResults(logdir / "result.json", vm["agents"].as<string>(),
                        vm["cbs_payment"].as<bool>());
        cbs.clearSearchEngines();
    }
    else if (algo == "PBS")
    {
        PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
        pbs.setSolverParams(vm["dummyStart"].as<bool>(),
                            vm["exhaustiveSearch"].as<bool>());

        // run
        // double runtime = 0;
        pbs.solve(vm["cutoffTime"].as<double>());
        // if (vm["saveStats"].as<bool>())
        //     pbs.saveResults((logdir / "stats.csv").string(),
        //                     vm["agents"].as<string>());
        if (pbs.solution_found && vm["savePath"].as<bool>())
            pbs.savePaths((logdir / "paths.txt").string());
        pbs.saveResults(logdir / "result.json", vm["agents"].as<string>());
        // pbs.savePriorityGraphs(logdir / "all_sols.json"); // for debug
        /*size_t pos = (logdir / "stats.csv").string().rfind('.');      //
        position of the file extension string output_name = (logdir /
        "stats.csv").string().substr(0, pos);     // get the name without
        extension cbs.saveCT(output_name); // for debug*/
        pbs.clearSearchEngines();
    }
    else if (algo == "IDPBS")
    {
        IDPBS idpbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>(),
                    vm["dummyStart"].as<bool>(),
                    vm["exhaustiveSearch"].as<bool>());
        idpbs.solve(vm["cutoffTime"].as<double>());
        if (idpbs.solution_found && vm["savePath"].as<bool>())
            idpbs.savePaths((logdir / "paths.txt").string());
        idpbs.saveResults(logdir / "result.json", vm["agents"].as<string>());
    }

    return 0;
}