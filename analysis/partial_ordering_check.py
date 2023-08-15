import os
import json


def find_consistent_graphs(total_order, extra_graph, sols):
    for sol in sols:
        for graph in sols[sol]:
            graph_ = [x.split("<") for x in graph.split(",")[:-1]]
            consistent = True
            for edge in graph_:
                a, b = edge # a should be lower than b
                if total_order.index(a) < total_order.index(b):
                    consistent = False
                    break
            if consistent and graph != extra_graph:
                print(f"graph {graph} is consistent with order {total_order}")



if __name__ == "__main__":
    with open("logs/2023-08-15_04-17-16_PBS_k=5_seed=8_aafe1e80-f94f-440d-9dfd-6784cb3eb9f0/all_sols.json") as f:
        new_sols = json.load(f)
    with open("logs/2023-08-15_04-17-16_PBS_k=5_seed=8_aafe1e80-f94f-440d-9dfd-6784cb3eb9f0/all_sols old.json") as f:
        old_sols = json.load(f)

    total_order = "24310" # smaller index has higher priority
    extra_graph = "0<1,0<2,1<3,1<4,3<2,3<4,4<2,"

    print("new sols: ")
    find_consistent_graphs(total_order, extra_graph, new_sols)
    print("old sols: ")
    find_consistent_graphs(total_order, extra_graph, old_sols)