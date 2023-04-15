import os
import fire
import json
import matplotlib.pyplot as plt


def plot_payment(logdir):
    with open(os.path.join(logdir, "result.json"), "r") as f:
        data = json.load(f)

    payments = data["payments"]

    # data to plot
    epsilon = 1e-7
    x = [x+epsilon for x in range(len(payments))]
    y = payments

    # create a figure and an Axes object
    fig, ax = plt.subplots(figsize=(12, 6))

    # create a bar plot using the Axes object
    ax.bar(x, y, width=0.6)

    # set the title and axis labels
    ax.set_title("Payments of Agents", fontsize=30)
    ax.set_xlabel("Agent ID", fontsize=20)
    ax.set_ylabel("Payment", fontsize=20)
    ax.set_ylim(-20, 20)

    # save the plot as a PNG image file
    fig.savefig(os.path.join(logdir, "payment.png"))

if __name__ == "__main__":
    fire.Fire(plot_payment)