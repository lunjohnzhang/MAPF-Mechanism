import os
import json
from pathlib import Path
import matplotlib.pyplot as plt
import argparse


def main(dir_path, num_agents, plot_extra_name):
    # Big array to collect all the payments
    payments_array = []

    # Looping through subdirectories
    for subdir, _, _ in os.walk(dir_path):
        result_file_path = os.path.join(subdir, 'result.json')

        # Check if the result.json file exists
        if os.path.isfile(result_file_path):
            with open(result_file_path, 'r') as file:
                data = json.load(file)

                # Get the payments
                payments = data.get('payments', [])

                # Check if the length of payments is equal to num_agents
                if len(payments) == num_agents:
                    payments_array += payments

    # Plotting the histogram
    plt.figure(figsize=(8, 6))
    plt.hist(payments_array, bins=20, edgecolor='black', log=True)
    plt.title(f'Histogram of Payments', fontsize=25)
    plt.xlabel('Payments', fontsize=23)
    plt.ylabel('Frequency', fontsize=23)
    plt.tick_params(axis='both', which='major', labelsize=18)
    plt.tick_params(axis='both', which='minor', labelsize=15)
    plt.tight_layout()
    plt.savefig(Path(args.dir_path) / Path(f'histogram_payments.pdf'))
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Process a directory and plot payments.')
    parser.add_argument('dir_path',
                        type=str,
                        help='Path to the directory containing subdirectories')
    parser.add_argument('num_agents',
                        type=int,
                        help='Number of agents for comparison')
    parser.add_argument('--plot-extra-name',
                        type=int,
                        help='extra stuff to put in plot')

    args = parser.parse_args()
    main(args.dir_path, args.num_agents, args.plot_extra_name)
