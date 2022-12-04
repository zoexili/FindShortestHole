import queue
import sys
import numpy as np
import simplegraphs as sg


def shortestHole(G, s):
    found = False
    hole_length = -1  # Default value for when no hole is found
    hole_nodes = []  # Default value for when no hole is found

    ########################################
    # Write code that finds the shortest hole containing s if one exists
    # If one exists, set 'found' to True
    # Set hole_length to be the length of the shortest hole
    # Set hole_nodes to be a list of the nodes in the hole in order,
    #    starting from s (and not repeating s)
    ########################################

    # For example, maybe your first step would be to run the usual BFS
    # G = sg.readGraph("inputs/G_10r.txt")
    # print(G["n"])  # number of nodes
    # print(G["m"])  # number of edges, need to /2
    # adjacency list of each node
    # print("adjacency list of each node", Graph)
    # print(G["adj"][0])  # adjacency list of node 0
    # sg.addUndirEdge(G, 0, 6)
    # print(G["adj"][0]) # adjacency list of node 0
    # print(6 in G["adj"][0]) # if 6 in adj list of node 0
    # print(2 in G["adj"][0])
    # for v in G["adj"][0]:
    #     print(v, "is a neighbor of node 0")
    distances, parents, layers = sg.BFS(G, s)
    Graph = G["adj"]
    # print(Graph)
    # print(layers[0])
    # print(layers[1])
    # print(layers[2])
    # print(layers[3])
    # print(distances) # distance from node 0 to other nodes
    # print(parents[1])
    # print(parents[2])
    # print(parents[3])
    # print(parents[4])
    # print(parents[5])
    # print(parents[6])
    # print(parents[9])
    firstNode = {}  # create an empty hashmap for branches
    # name each branch's parent node in firstNode hash table, starting from second layer
    for u in layers[1]:
        firstNode[u] = u

    # name each branch's nodes as their branch's parent node, starting from third layer.
    for i in range(2, len(layers)):
        for v in layers[i]:
            firstNode[v] = firstNode[parents[v]]
    # print("firstNode: ", firstNode)

    # initialize boolean found to false and hole length to positive infinity
    sim_cyc_found = False
    # this parameter was set incorrectly to sim_cyc_length. It should be hole_length
    hole_length = float('inf')

    def BFSpath(graph, start, end):
        parent = {}
        q = queue.Queue()
        q.put(start)
        parent[start] = None
        while not (q.empty()):
            u = q.get()
            # print("u: ", u)
            if u == end:
                # assign path to the end node
                path = [end]
                # print("path: ", path)
                # when the last node in the path is not the source node, add the last node's parent in the path
                while path[-1] != start:
                    path.append(parent[path[-1]])
                    # print("path after append: ", path)
                # reverse the path since we were backtracing from end node to start node
                path.reverse()
                # print("reversed path: ", path)
                return path
            # check the neighbors v of u
            for v in graph.get(u):
                # print("v in graph.get(u): ", v)
                # if v is not in parent, record it and add v to q
                if v not in parent:
                    # print("v not in parent: ", v)
                    parent[v] = u
                    # print("parents[v]: ", parents[v])
                    q.put(v)

    # print(BFSpath(Graph, 0, 6))

    # Look for the first edge that crosses between branches of BFS tree, starting from second layer
    # Once we completely checked a layer, see if a cycle was found
    # when there is a reference before assignment error, initialize the variable first.
    sim_cyc_edge = (0, 0)
    for i in range(1, len(layers)):
        for u in layers[i]:
            for v in G["adj"][u]:
                # if v is not the source vertex, and u and v are in two different branches
                if v != s and firstNode[u] != firstNode[v]:
                    sim_cyc_found = True
                    this_cyc_len = distances[u] + distances[v] + 1
                    if this_cyc_len < hole_length:
                        # this needs to be in the if loop so that you only record the edge when the hole length is smaller.
                        sim_cyc_edge = (u, v)
                        hole_length = this_cyc_len
        if sim_cyc_found:  # this loop is under for v loop, it should be under for u, bc we need to go through all the u's neighbors.
            part1 = BFSpath(Graph, s, sim_cyc_edge[0])
            part2 = BFSpath(Graph, s, sim_cyc_edge[1])
            part2.reverse()
            part2.remove(s)
            hole_nodes = part1 + part2
            return True, hole_length, hole_nodes

    # Return the output
    return found, -1, hole_nodes


#########################################################
# Don't modify the stuff below this line for submission
# (Of course you can change it while you're
#    doing your own testing if you want)
#########################################################


def readSource(start_file):
    # The source vertex is listed in its own file
    # It is an integer on a line by itself.
    with open(start_file, 'r') as f:
        raw_start = f.readline()
        s = int(raw_start)
    return s


def writeOutput(output_file, hole_found, hole_length, hole_list):
    # This takes the outputs of shortestHole and writes them
    # to a file with the name output_file
    with open(output_file, 'w') as f:
        f.write("{}\n".format(hole_found))
        f.write("{}\n".format(hole_length))
        f.write("{}\n".format(hole_list))
    return


def main(args=[]):
    # Expects three command-line arguments:
    # 1) name of a file describing the graph
    # 2) name of a file with the ID of the start node
    # 3) name of a file where the output should be written
    if len(args) != 3:
        print("Problem! There were {} arguments instead of 3.".format(len(args)))
        return
    graph_file = args[0]
    start_file = args[1]
    out_file = args[2]
    G = sg.readGraph(graph_file)  # Read the graph from disk
    s = readSource(start_file)  # Read the source from disk
    hole_found, hole_length, hole_list = shortestHole(
        G, s)  # Find the shortest hole!
    writeOutput(out_file, hole_found, hole_length,
                hole_list)  # Write the output
    return


if __name__ == "__main__":
    main(sys.argv[1:])
