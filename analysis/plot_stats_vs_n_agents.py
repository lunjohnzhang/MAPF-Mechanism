import os
import json
import fire

import numpy as np
import scipy.stats as st
import matplotlib.pyplot as plt

from dataclasses import dataclass, fields

FIELD_TO_LABEL = {
    "runtime": "Runtime",
    "success": "Success Rate",
}


@dataclass
class Stats:
    runtime: float = None
    success: int = 0


def add_to_dict(key, val, the_dict):
    if key in the_dict:
        the_dict[key].append(val)
    else:
        the_dict[key] = [val]


def plot_stats_single(logdirs, to_plot, field_name, algo, ax=None):
    """Convert dict of Stats to numpy array and plot"""
    save_fig = False
    if ax is None:
        save_fig = True
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    to_plot = sorted(to_plot.items())
    agent_nums = [agent_num for agent_num, _ in to_plot]
    # for field in fields(Stats):
    all_vals = []
    for n_agent, stats in to_plot:
        # print(n_agent)
        all_vals.append([getattr(stat, field_name) for stat in stats])

    all_vals = np.array(all_vals, dtype=float)

    if field_name in ["runtime"]:
        # Plot mean and 95% cf
        mean_vals = np.mean(all_vals, axis=1)
        cf_vals = st.t.interval(confidence=0.95,
                                df=all_vals.shape[1] - 1,
                                loc=mean_vals,
                                scale=st.sem(all_vals, axis=1) + 1e-8)

        ax.plot(
            agent_nums,
            mean_vals,
            marker=".",
            # color=color,
            label=algo,
            # label=f"{map_from}",
        )
        ax.fill_between(
            agent_nums,
            cf_vals[1],
            cf_vals[0],
            alpha=0.5,
            # color=color,
        )

    elif field_name in ["success"]:
        # plot success rate
        success_rate = np.sum(all_vals, axis=1) / all_vals.shape[1]

        ax.plot(
            agent_nums,
            success_rate,
            marker=".",
            # color=color,
            label=algo,
            # label=f"{map_from}",
        )

    if save_fig:
        ax.set_ylabel(FIELD_TO_LABEL[field_name], fontsize=25)
        ax.set_xlabel("Number of Agents", fontsize=25)
        # ax.set_ylim(y_min, y_max)
        # ax.grid()
        ax.tick_params(axis='both', which='major', labelsize=20)
        ax.tick_params(axis='both', which='minor', labelsize=15)

        ax.figure.tight_layout()
        fig.savefig(
            os.path.join(
                logdirs,
                f"{field_name}.png",
            ),
            dpi=300,
        )


def collect_results(logdirs):
    # All results to plot, key is the current algo, value is the `to_plot` dict
    # of the current algo
    print(f"Collecting results")

    to_plot_algo = {}
    # all_logdir_algo_f = []

    # Loop through all logdirs to get the stats
    for logdir_algo in os.listdir(logdirs):
        logdir_algo_f = os.path.join(logdirs, logdir_algo)
        if not os.path.isdir(logdir_algo_f):
            continue

        # all_logdir_algo_f.append(logdir_algo_f)
        current_algo = None

        # All results to plot for currrent algo, key is n_agents, value are
        # list of Stats
        to_plot = {}
        for logdir in os.listdir(logdir_algo_f):
            logdir_f = os.path.join(logdir_algo_f, logdir)
            if not os.path.isdir(logdir_f):
                continue

            # Read in config and results
            with open(os.path.join(logdir_f, "config.json"), "r") as f:
                config = json.load(f)
            with open(os.path.join(logdir_f, "result.json"), "r") as f:
                result = json.load(f)

            current_algo = config["algo"]

            if current_algo == "ECBS" and config["highLevelSolver"] == "EES" :
                current_algo = "EECBS"
            elif current_algo == "PP":
                if logdir_algo == "PP_A*":
                    current_algo = "Monte Carlo PP (A*)"
                else:
                    current_algo = "Monte Carlo PP"
            elif current_algo == "PP1":
                current_algo = "First Come First Serve"
            elif current_algo == "PBS" and config["exhaustiveSearch"]:
                current_algo = "Exhaustive PBS"

            n_agents = config["agentNum"]

            # success? We have well-formed instances, so the only way to fail
            # is time/node out.
            success = 0
            if (not result["timeout"]
                    and not ("nodeout" in result and result["nodeout"])):
                success = 1

            stat = Stats(runtime=result["runtime"], success=success)

            add_to_dict(n_agents, stat, to_plot)

        to_plot_algo[(current_algo, logdir_algo_f)] = to_plot
    return to_plot_algo


def main(logdirs, add_legend=True):
    to_plot_algo = collect_results(logdirs)

    for field in fields(Stats):
        print(f"Plotting {field.name}")

        fig, ax = plt.subplots(1, 1, figsize=(8, 8))

        for algo, to_plot in to_plot_algo.items():
            algo_name, logdir_algo_f = algo
            plot_stats_single(logdir_algo_f, to_plot, field.name, algo_name,
                              ax)

        # Post process
        ax.set_ylabel(FIELD_TO_LABEL[field.name], fontsize=25)
        ax.set_xlabel("Number of Agents", fontsize=25)
        # ax.set_ylim(y_min, y_max)
        # ax.grid()
        ax.tick_params(axis='both', which='major', labelsize=20)
        ax.tick_params(axis='both', which='minor', labelsize=15)

        if add_legend:
            handles, labels = ax.get_legend_handles_labels()
            legend = ax.legend(
                handles,
                labels,
                loc="lower left",
                ncol=1,
                fontsize=25,
                mode="expand",
                bbox_to_anchor=(0, 1.02, 1, 0.2),  # for ncols=2
                # borderaxespad=0,)
            )
            for line in legend.get_lines():
                line.set_linewidth(4.0)

        ax.figure.tight_layout()

        fig.savefig(
            os.path.join(
                logdirs,
                f"{field.name}.png",
            ),
            dpi=300,
        )


if __name__ == "__main__":
    fire.Fire(main)