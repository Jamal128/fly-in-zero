from parser import Parser
from graph import Graph
from time_expanded_graph import TimeExpandedGraph


def main():
    parser = Parser()
    max_turns = 55
    graph, nb_drones = parser.parse_file("maps/challenger/01_the_impossible_dream.txt")
    teg = TimeExpandedGraph(graph, max_turns, nb_drones)
    mcf = teg.build()               # construye el TEG

    flow, cost = mcf.run(
        teg.source_id,
        teg.sink_id,
        nb_drones,
    )

    print("flow =", flow)
    print("cost =", cost)

    paths = teg.extract_paths(mcf)

    for i, path in enumerate(paths, start=1):
        print(f"Drone {i}: {path}")

if __name__ == "__main__":
    main()