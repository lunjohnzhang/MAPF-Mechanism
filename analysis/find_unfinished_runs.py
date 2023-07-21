import os
import json
import fire


def find_unfinished_runs(logdirs):
    # Loop through all logdirs to find unfinished runs
    for logdir_algo in os.listdir(logdirs):
        logdir_algo_f = os.path.join(logdirs, logdir_algo)
        if not os.path.isdir(logdir_algo_f):
            continue

        for logdir in os.listdir(logdir_algo_f):
            logdir_f = os.path.join(logdir_algo_f, logdir)
            if not os.path.isdir(logdir_f):
                continue

            try:
                # Read in config and results
                with open(os.path.join(logdir_f, "config.json"), "r") as f:
                    config = json.load(f)
                with open(os.path.join(logdir_f, "result.json"), "r") as f:
                    result = json.load(f)
            except FileNotFoundError:
                print(logdir_f)

if __name__ == "__main__":
    fire.Fire(find_unfinished_runs)