import os
import fire
import json

import numpy as np


def gen_cost_value(
    n_agent=10,
    n_instances=100,
    cost_mode="uniform",
    value_mode="uniform",
    force_save=False,
):
    cost_store_dir = os.path.join("config", "agent_costs", cost_mode)
    value_store_dir = os.path.join("config", "agent_values", value_mode)
    os.makedirs(cost_store_dir, exist_ok=True)
    os.makedirs(value_store_dir, exist_ok=True)
    for i in range(1, n_instances + 1):
        cost_save_path = os.path.join(cost_store_dir, f"{n_agent}_{i}.json")
        value_save_path = os.path.join(value_store_dir, f"{n_agent}_{i}.json")

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