import os
import fire
import json

import numpy as np


def gen_cost_value(
    n_agent=10,
    cost_mode="uniform",
    value_mode="uniform",
    force_save=False,
):
    cost_save_path = os.path.join("config", "agent_costs",
                                  f"{cost_mode}_{n_agent}.json")
    value_save_path = os.path.join("config", "agent_values",
                                   f"{value_mode}_{n_agent}.json")

    if os.path.exists(cost_save_path) and not force_save:
        print("Specified cost config already exists, skip")
    else:
        costs = None
        # Sample from uniform distribution between 0 and 1
        if cost_mode == "uniform":
            costs = np.random.uniform(0, 1, size=n_agent)
        # With categorical distribution, flip a coin to decide low cost agents
        # (0.5) and high cost agents (1)
        elif cost_mode == "categorical":
            flip_coins = np.random.uniform(0, 1, size=n_agent) < 0.5
            costs = np.ones(n_agent)
            costs[flip_coins] = 0.5
        elif cost_mode == "all_one":
            costs = np.ones(n_agent)
        else:
            raise ValueError(f"Unknown cost mode \"{cost_mode}\"")

        # Write costs to config
        with open(cost_save_path, "w") as f:
            json.dump(
                {
                    "mode": cost_mode,
                    "vals": costs.tolist(),
                },
                f,
                indent=4,
            )

    if os.path.exists(value_save_path) and not force_save:
        print("Specified value config already exists, skip")
    else:
        values = None
        if value_mode == "uniform":
            values = np.random.uniform(0, 1, size=n_agent)
        else:
            raise ValueError(f"Unknown value mode \"{value_mode}\"")
        # Write values to config
        with open(value_save_path, "w") as f:
            json.dump(
                {
                    "mode": value_mode,
                    "vals": values.tolist(),
                },
                f,
                indent=4,
            )


if __name__ == "__main__":
    fire.Fire(gen_cost_value)