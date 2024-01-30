import os
import json
import fire
import shutil

import numpy as np
import scipy.stats as st
import matplotlib.pyplot as plt
import matplotlib.scale as mscale
import matplotlib.transforms as mtransforms

from dataclasses import dataclass, fields

FIELD_TO_LABEL = {
    "runtime": "Runtime",
    "success": "Success Rate",
    "solution_cost": "Solution Cost",
    "social_welfare": "Social Welfare",
    "social_welfare_subopt": "SW Div w/ FCFS",
    "solution_cost_subopt": "Solution Cost Diff w/ FCFS",
    "std_payment": "Payment Std",
}

ALGO_TO_COLOR_MARKER = {
    "EECBS": ("green", "o"),  # circle
    "PCBS": ("red", "^"),  # triangle_up
    "PCBS (w/o payment)": ("deepskyblue", "D"), # diamond
    "Monte Carlo PP (100)": ("orange", "s"),  # square
    "Monte Carlo PP (10)": ("green", "v"),  # triangle_down
    "Monte Carlo PP (50)": ("olive", "X"),  # X (filled)
    "Monte Carlo PP (150)": ("pink", "h"),  # Hexagon
    "First-Come-First-Serve": ("blue", "P"),  # plus
    "Exhaustive PBS": ("purple", "*"),  # star
}

FIRST_LARGE_X = 25


class CustomTransform(mtransforms.Transform):
    input_dims = 1
    output_dims = 1
    is_separable = True

    def __init__(self, first_large_x):
        super().__init__()
        self.first_large_x = first_large_x

    def transform_non_affine(self, x):
        return np.piecewise(x, [x <= 25, x > 25], [
            lambda x: x, lambda x: (15 / (self.first_large_x - 25)) *
            (x - 25) + 25
        ])

    def inverted(self):
        return CustomTransformInverted(first_large_x=self.first_large_x)


class CustomTransformInverted(mtransforms.Transform):
    input_dims = 1
    output_dims = 1
    is_separable = True

    def __init__(self, first_large_x):
        super().__init__()
        self.first_large_x = first_large_x

    def transform_non_affine(self, x):
        return np.piecewise(x, [x <= 25, x > 25], [
            lambda x: x, lambda x: (self.first_large_x - 25) / 15 *
            (x - 25) + 25
        ])

    def inverted(self):
        return CustomTransform()


class CustomScale(mscale.ScaleBase):
    name = 'custom'

    def __init__(self, axis, first_large_x=620, **kwargs):
        super().__init__(axis)
        self.first_large_x = first_large_x

    def get_transform(self):
        return CustomTransform(first_large_x=self.first_large_x)

    def set_default_locators_and_formatters(self, axis):
        pass


mscale.register_scale(CustomScale)


@dataclass
class Stats:
    runtime: float = None
    success: int = 0
    solution_cost: float = None
    social_welfare: float = None

    # Standard deviation of the payments of the agents. It measures the
    # "fairness" of the algorithm.
    std_payment: float = None

    # Diff solution_cost (OUR_ALGO - BASELINE)
    solution_cost_subopt: float = None

    # Diff social_welfare (OUR_ALGO - BAELINE)
    social_welfare_subopt: float = None


def add_to_dict(key, val, seed, the_dict):
    if key in the_dict:
        the_dict[key][seed] = val
    else:
        # Initialize the sub-dictionary
        the_dict[key] = {seed: val}
        # the_dict[key] = [val]


def plot_stats_single(
    logdirs,
    to_plot,
    field_name,
    algo,
    ax=None,
    plot_only_success=False,
    n_runs_mcpp=100,
):
    """Convert dict of Stats to numpy array and plot"""

    # Ignore payment plotting for CBS and EECBS
    if algo in ["CBS", "EECBS"] and field_name in ["std_payment"]:
        return None

    save_fig = False
    if ax is None:
        save_fig = True
        fig, ax = plt.subplots(1, 1, figsize=(8, 5.5))

    to_plot = sorted(to_plot.items())
    agent_nums = []
    # for field in fields(Stats):
    all_vals = []
    for n_agent, stats in to_plot:
        # print(n_agent)
        curr_vals = []
        breakout = False
        # For solution cost, ignore the entry if success rate is not 100%
        if field_name in [
                "solution_cost",
                "social_welfare",
                "social_welfare_subopt",
                "solution_cost_subopt",
                "std_payment",
        ]:
            for _, stat in stats.items():
                if stat.success == 1:
                    curr_vals.append(getattr(stat, field_name))
                # if stat.success == 0:
                #     # should be ignored
                #     breakout = True
                #     break

            if len(curr_vals) < 100 and not plot_only_success:
                break
            if (len(curr_vals) > 0):
                agent_nums.append(n_agent)
                all_vals.append(curr_vals)
        else:
            agent_nums.append(n_agent)
            all_vals.append(
                [getattr(stat, field_name) for _, stat in stats.items()])

            if field_name == "runtime":
                for i in range(len(all_vals)):
                    while len(all_vals[i]) < 100:
                        all_vals[i].append(3600)

    if len(all_vals) == 0:
        return
    # print(all_vals)
    # all_vals = np.array(all_vals, dtype=float)

    agent_nums = np.array(agent_nums)
    normalized_x = range(len(agent_nums))

    # global FIRST_LARGE_X
    # FIRST_LARGE_X = max(FIRST_LARGE_X, agent_nums[np.argmax(agent_nums > 25)])
    # ax.set_xscale('custom', first_large_x=FIRST_LARGE_X)

    # Avoid scientific notation
    ax.get_xaxis().get_major_formatter().set_scientific(False)
    ax.get_yaxis().get_major_formatter().set_scientific(False)
    ax.ticklabel_format(useOffset=False)

    color, marker = ALGO_TO_COLOR_MARKER[algo]

    if field_name in [
            "runtime",
            "solution_cost",
            "social_welfare",
            "social_welfare_subopt",
            "solution_cost_subopt",
            "std_payment",
    ]:
        # Plot mean and 95% cf
        mean_vals = [np.mean(curr_vals) for curr_vals in all_vals]
        cf_vals = []
        for i, curr_vals in enumerate(all_vals):
            cf_val = st.t.interval(confidence=0.95,
                                   df=len(curr_vals) - 1,
                                   loc=mean_vals[i],
                                   scale=st.sem(curr_vals) + 1e-8)
            cf_vals.append(cf_val)
        cf_vals = np.array(cf_vals).T
        # mean_vals = np.mean(all_vals, axis=1)
        # cf_vals = st.t.interval(confidence=0.95,
        #                         df=all_vals.shape[1] - 1,
        #                         loc=mean_vals,
        #                         scale=st.sem(all_vals, axis=1) + 1e-8)
        label = algo
        if algo == "Monte Carlo PP":
            label = f"{algo} ({n_runs_mcpp})"

        # print(mean_vals)
        # print(st.sem(curr_vals))
        ax.plot(
            normalized_x,
            mean_vals,
            marker=marker,
            color=color,
            label=label,
            markersize=15,
        )
        ax.fill_between(
            normalized_x,
            cf_vals[1],
            cf_vals[0],
            alpha=0.5,
            color=color,
        )

    elif field_name in ["success"]:
        # plot success rate
        try:
            all_vals = np.array(all_vals, dtype=float)
        except ValueError:
            max_len = np.max([len(x) for x in all_vals])
            for i in range(len(all_vals)):
                while len(all_vals[i]) < max_len:
                    all_vals[i].append(0)
            all_vals = np.array(all_vals, dtype=float)
        success_rate = np.sum(all_vals, axis=1) / all_vals.shape[1]
        ax.plot(
            normalized_x,
            success_rate,
            marker=marker,
            color=color,
            label=algo,
            markersize=15,
        )
        ax.set_ylim(-0.1, 1.1)

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
        fig.savefig(
            os.path.join(
                logdirs,
                f"{field_name}.pdf",
            ),
            dpi=300,
        )
    return agent_nums


def collect_results(logdirs, baseline_algo="PP1"):
    # All results to plot, key is the current algo, value is the `to_plot` dict
    # of the current algo
    print(f"Collecting results")

    to_plot_algo = {}

    # Obtain all_logdir_algo.
    all_logdir_algo = os.listdir(logdirs)

    # Plot PBS last (or second to last is there is CBS) so it's visible
    if "PBS" in all_logdir_algo:
        all_logdir_algo.remove("PBS")
        all_logdir_algo.append("PBS")

    # Plot CBS last so it's visible
    if "CBS" in all_logdir_algo:
        all_logdir_algo.remove("CBS")
        all_logdir_algo.append("CBS")

    # We want to collect results for `baseline_algo` first to compute
    # suboptimalities
    all_logdir_algo.remove(baseline_algo)
    all_logdir_algo.insert(0, baseline_algo)
    baseline_algo_name = None
    baseline_logdir_algo_f = None

    # Loop through all logdirs to get the stats
    for logdir_algo in all_logdir_algo:
        logdir_algo_f = os.path.join(logdirs, logdir_algo)
        if not os.path.isdir(logdir_algo_f):
            continue

        # all_logdir_algo_f.append(logdir_algo_f)
        current_algo = None

        # All results to plot for currrent algo, key is n_agents, value are
        # list of Stats indexed by the seed. The same seed corresponds to the
        # same scen and same cost/value configs.
        to_plot = {}
        for logdir in os.listdir(logdir_algo_f):
            logdir_f = os.path.join(logdir_algo_f, logdir)
            if not os.path.isdir(logdir_f):
                continue

            result_exist = True
            # Read in config and results
            with open(os.path.join(logdir_f, "config.json"), "r") as f:
                config = json.load(f)
            try:
                with open(os.path.join(logdir_f, "result.json"), "r") as f:
                    result = json.load(f)
            except json.decoder.JSONDecodeError:
                breakpoint()
            except FileNotFoundError:
                result_exist = False

            current_algo = config["algo"]

            if current_algo == "ECBS" and config["highLevelSolver"] == "EES":
                current_algo = "EECBS"
            elif current_algo == "PP":
                if logdir_algo == "PP_A*":
                    current_algo = "Monte Carlo PP (A*)"
                else:
                    n_runs = config["nRuns"]
                    current_algo = f"Monte Carlo PP ({n_runs})"
            elif current_algo == "PP1":
                current_algo = "First-Come-First-Serve"
            elif current_algo == "PBS" and config["exhaustiveSearch"]:
                current_algo = "Exhaustive PBS"
            elif current_algo == "CBS":
                calc_payment = config[
                    "cbs_payment"] if "cbs_payment" in config else True
                current_algo = "PCBS" if calc_payment else "PCBS (w/o payment)"

            n_agents = config["agentNum"]
            seed = config["seed"]

            if result_exist:
                # success? We have well-formed instances, so the only way to
                # fail is time/node out if result exist
                success = 0
                if (not result["timeout"]
                        and not ("nodeout" in result and result["nodeout"])):
                    success = 1

                # Social welfare is sum of values - solution cost
                # values = result["values"]
                solution_cost = result["solution_cost"]
                social_welfare = result[
                    "social_welfare"] if "social_welfare" in result else np.inf
                if not success and social_welfare == 0:
                    social_welfare = np.inf

                # Compute std of payments
                std_payment = None
                if "payments" in result:
                    if current_algo == "CBS":
                        payment_calc_success = result[
                            "payment_calculate_success"]
                        if payment_calc_success:
                            payments = result["payments"]
                            std_payment = np.std(payments)
                        else:
                            payments = None
                    else:
                        payments = result["payments"]
                        try:
                            std_payment = np.std(payments)
                        except TypeError:
                            print(
                                f"{current_algo}: Payment not proper: {logdir_f}"
                            )
                            shutil.rmtree(logdir_f)

                runtime = result["runtime"]
                if current_algo == "CBS" and success == 1:
                    runtime = result["total_runtime"]
            # Solution not exist
            else:
                runtime = config["cutoffTime"]
                solution_cost = -1
                social_welfare = np.inf
                success = 0

            social_welfare_subopt = None
            solution_cost_subopt = None

            # Compute suboptimalities
            if logdir_algo == baseline_algo:
                social_welfare_subopt = 1
                solution_cost_subopt = 0
                baseline_algo_name = current_algo
                baseline_logdir_algo_f = logdir_algo_f
            else:
                try:
                    baseline_stat = to_plot_algo[(
                        baseline_algo_name,
                        baseline_logdir_algo_f)][n_agents][seed]
                    social_welfare_subopt = social_welfare / baseline_stat.social_welfare
                    solution_cost_subopt = solution_cost - baseline_stat.solution_cost
                except RuntimeWarning:
                    breakpoint()

            # if current_algo == "CBS" and not success:
            #     print(logdir_f)

            stat = Stats(runtime=runtime,
                         success=success,
                         solution_cost=solution_cost,
                         social_welfare=social_welfare,
                         social_welfare_subopt=social_welfare_subopt,
                         solution_cost_subopt=solution_cost_subopt,
                         std_payment=std_payment)

            add_to_dict(n_agents, stat, seed, to_plot)

        to_plot_algo[(current_algo, logdir_algo_f)] = to_plot
    return to_plot_algo


def main(logdirs, add_legend=True, legend_only=False, baseline_algo="PP1"):
    to_plot_algo = collect_results(logdirs, baseline_algo=baseline_algo)

    ###################### For debugging ######################
    # pp = to_plot_algo[('Monte Carlo PP', 'logs/to_show/demo/PP')]
    # pbs = to_plot_algo[('Exhaustive PBS', 'logs/to_show/demo/PBS')]
    # for i in range(1, 101):
    #     assert pp[25][i].solution_cost + 1e-3 >= pbs[25][i].solution_cost
    #     print(f"Seed {i}, pp={pp[25][i].solution_cost}, pbs={pbs[25][i].solution_cost}")
    # exit()

    ###################### For debugging ######################

    for field in fields(Stats):
        print(f"Plotting {field.name}")

        figsize = (10, 5.5)
        if legend_only:
            figsize = (22, 8)

        fig, ax = plt.subplots(1, 1, figsize=figsize)

        longest_agent_nums = None

        for algo, to_plot in to_plot_algo.items():
            algo_name, logdir_algo_f = algo
            agent_nums = plot_stats_single(logdir_algo_f, to_plot, field.name,
                                           algo_name, ax)

            # We want the longest because some agent_nums might be incomplete
            # because of failed runs
            if agent_nums is not None and (longest_agent_nums is None
                                           or len(longest_agent_nums)
                                           < len(agent_nums)):
                longest_agent_nums = agent_nums

        # Post process
        ax.set_ylabel(FIELD_TO_LABEL[field.name], fontsize=32)
        ax.set_xlabel("Number of Agents", fontsize=32)

        if longest_agent_nums is not None:
            normalized_x = range(len(longest_agent_nums))

            ax.set_xticks(normalized_x)
            ax.set_xticklabels(longest_agent_nums)

        # ax.set_ylim(y_min, y_max)
        # ax.grid()
        ax.tick_params(axis='both', which='major', labelsize=25)
        ax.tick_params(axis='both', which='minor', labelsize=15)

        if add_legend:
            ncol = 2
            handles, labels = ax.get_legend_handles_labels()

            if legend_only:
                ncol = len(ALGO_TO_COLOR_MARKER.keys())

                # The desired order of legend items
                custom_order = [0, 3, 1, 2]  # For comparing all algos
                custom_order = [0, 3, 4, 1, 2]  # For comparing MCPP
                handles = [handles[i] for i in custom_order]
                labels = [labels[i] for i in custom_order]

            legend = ax.legend(
                handles,
                labels,
                loc="lower left",
                ncol=ncol,
                fontsize=20,
                mode="expand",
                bbox_to_anchor=(0, 1.02, 1, 0.2),  # for ncols=2
                # borderaxespad=0,)
            )
            for line in legend.get_lines():
                line.set_linewidth(2.0)

        # ax.figure.tight_layout()
        fig.tight_layout()
        fig.savefig(
            os.path.join(
                logdirs,
                f"{field.name}.png",
            ),
            dpi=300,
        )

        fig.savefig(
            os.path.join(
                logdirs,
                f"{field.name}.pdf",
            ),
            dpi=300,
        )

        if legend_only:
            break


if __name__ == "__main__":
    fire.Fire(main)
