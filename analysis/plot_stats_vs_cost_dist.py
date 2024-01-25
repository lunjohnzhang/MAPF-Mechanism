import os
import json
import fire
import shutil
import yaml

import numpy as np
import scipy.stats as st
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.scale as mscale
import matplotlib.transforms as mtransforms

from dataclasses import dataclass, fields

from plot_stats_vs_n_agents import collect_results, ALGO_TO_COLOR_MARKER

mpl.use("agg")
plt.rc("text", usetex=True)


def get_cost_value_label(raw_label):
    if raw_label == "uniform":
        return "Uniform"
    elif "logN" in raw_label:
        _, mu, sigma = raw_label.split("-")
        return rf"$\mathcal{{LN}}({mu}, {sigma}^2)$"


def plot_stats_vs_cost_dist(logdirs, n_agents=1800):
    ticks_labels = []
    all_welfare_subopt = []
    for logdir in os.listdir(logdirs):
        logdir = os.path.join(logdirs, logdir)
        if not os.path.isdir(logdir):
            continue

        # Collect results
        curr_result = collect_results(logdir, baseline_algo="PP1")

        # Get meta info
        with open(os.path.join(logdir, "meta.yaml"), "r") as f:
            meta = yaml.safe_load(f)

        value_label = get_cost_value_label(meta["value"])
        cost_label = get_cost_value_label(meta["cost"])

        ticks_labels.append(cost_label)

        # Get the suboptimality
        for (algo, logdir_algo_f) in curr_result:
            if algo == "Monte Carlo PP (100)":
                result = curr_result[(algo, logdir_algo_f)][n_agents]
                welfare_subopts = []
                for _, stats in result.items():
                    welfare_subopts.append(stats.social_welfare_subopt)
        all_welfare_subopt.append(welfare_subopts)

    all_welfare_subopt = np.array(all_welfare_subopt)
    mean_vals = np.mean(all_welfare_subopt, axis=1)
    cf_vals = st.t.interval(confidence=0.95,
                            df=all_welfare_subopt.shape[1] - 1,
                            loc=mean_vals,
                            scale=st.sem(all_welfare_subopt, axis=1) + 1e-8)

    fig, ax = plt.subplots(1, 1, figsize=(12, 8))

    color, marker = ALGO_TO_COLOR_MARKER[algo]

    ax.plot(
        np.arange(len(mean_vals)),
        mean_vals,
        marker=marker,
        color=color,
        # label=label,
        markersize=15,
    )
    ax.fill_between(
        np.arange(len(mean_vals)),
        cf_vals[1],
        cf_vals[0],
        alpha=0.5,
        color=color,
    )

    ax.tick_params(axis='y', labelsize=32)
    ax.set_xticks(np.arange(len(ticks_labels)))
    ax.set_xticklabels(ticks_labels, fontsize=25)
    ax.set_xlabel("Cost Distribution", fontsize=40)
    ax.set_ylabel("SW Div w/ FCFS", fontsize=40)

    fig.tight_layout()
    fig.savefig(
        os.path.join(logdirs, "welfare_subopt_vs_cost_dist.png"),
        dpi=300,
        bbox_inches='tight',
    )
    fig.savefig(
        os.path.join(logdirs, "welfare_subopt_vs_cost_dist.pdf"),
        dpi=300,
        bbox_inches='tight',
    )


if __name__ == "__main__":
    fire.Fire(plot_stats_vs_cost_dist)
