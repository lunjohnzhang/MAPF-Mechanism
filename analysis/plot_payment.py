import os
import fire
import json
import matplotlib.pyplot as plt


def plot_payments_histogram(payments, algo_name, save_path):
    plt.figure(figsize=(8, 5.5))
    plt.hist(payments, edgecolor='black', alpha=0.7)
    plt.title(f'Payments Distribution for {algo_name} Across All Runs',
              fontsize=15)
    plt.xlabel('Payment Value', fontsize=25)
    plt.ylabel('Frequency', fontsize=25)
    plt.xlim(0, None)
    plt.tick_params(axis='both', which='major', labelsize=20)
    plt.tick_params(axis='both', which='minor', labelsize=15)
    plt.grid(axis='y', alpha=0.75)
    plt.tight_layout()
    plt.savefig(os.path.join(save_path, f'{algo_name}_payments_histogram.png'))
    plt.savefig(os.path.join(save_path, f'{algo_name}_payments_histogram.pdf'))
    plt.close()


def plot_payment(logdir):
    with open(os.path.join(logdir, "config.json"), "r") as f:
        config = json.load(f)
    with open(os.path.join(logdir, "result.json"), "r") as f:
        result = json.load(f)

    current_algo = config["algo"]
    payments = result["payments"]

    if current_algo == "PP":
        current_algo = "Monte Carlo PP"
    elif current_algo == "PP1":
        current_algo = "First Come First Serve"
    elif current_algo == "PBS" and config["exhaustiveSearch"]:
        current_algo = "Exhaustive PBS"

    plot_payments_histogram(payments, current_algo, logdir)

    # # data to plot
    # epsilon = 1e-7
    # x = [x+epsilon for x in range(len(payments))]
    # y = payments

    # # create a figure and an Axes object
    # fig, ax = plt.subplots(figsize=(12, 6))

    # # create a bar plot using the Axes object
    # ax.bar(x, y, width=0.6)

    # # set the title and axis labels
    # ax.set_title("Payments of Agents", fontsize=30)
    # ax.set_xlabel("Agent ID", fontsize=20)
    # ax.set_ylabel("Payment", fontsize=20)
    # ax.set_ylim(-5, 5)

    # # save the plot as a PNG image file
    # fig.savefig(os.path.join(logdir, "payment.png"))


if __name__ == "__main__":
    fire.Fire(plot_payment)