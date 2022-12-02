import os
import fire
import json
import pandas as pd
import numpy as np

def main(log_dir):
    for sub_dir in os.listdir(log_dir):
        log_dir_sub_full = os.path.join(log_dir, sub_dir)
        all_log_files = os.listdir(log_dir_sub_full)
        if len(all_log_files) > 1:
            # Get min f
            metrics = {}
            result_file = os.path.join(log_dir_sub_full, "result.csv")
            result_df = pd.read_csv(result_file)
            min_f_vals = result_df["min f value"]
            min_f = min_f_vals[0]
            assert np.all(min_f_vals.to_numpy() == min_f)
            all_suboptimality = []
            for log_file in all_log_files:
                if "path" in log_file:
                    log_file_full = os.path.join(log_dir_sub_full, log_file)
                    with open(log_file_full, "r") as f:
                        paths = f.readlines()
                    total_len = 0
                    for path in paths:
                        pure_path = path.split(":")[-1].strip()
                        pure_path_len = len(pure_path.split("->")) - 2
                        total_len += pure_path_len
                    suboptimality = total_len/min_f
                    all_suboptimality.append(suboptimality)

            avg_suboptimality = np.mean(all_suboptimality)
            metrics["avg_suboptimality"] = avg_suboptimality
            with open(os.path.join(log_dir_sub_full, "metrics.json"), "w") as f:
                json.dump(metrics, f, indent=4)

if __name__ == '__main__':
    fire.Fire(main)