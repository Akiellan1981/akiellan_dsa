import heapq

class Node:
    def __init__(self, node, distance, path=[]):
        self.node = node
        self.distance = distance
        self.path = path

    def __lt__(self, other):
        return self.distance < other.distance

def print_path(mapping, target):
    path = mapping[target].path
    return path

def dijkstra(V, adj, S):
    visited = set()  # Track visited nodes for Dijkstra's algorithm
    mapping = {}
    q = []

    mapping[S] = Node(S, 0, path=[S])
    heapq.heappush(q, Node(S, 0))

    while q:
        n = heapq.heappop(q)
        v = n.node
        distance = n.distance
        path = n.path
        visited.add(v)

        adjList = adj[v]
        for adjLink in adjList:
            if adjLink[0] not in visited:
                new_path = path + [adjLink[0]]
                if adjLink[0] not in mapping:
                    mapping[adjLink[0]] = Node(adjLink[0], distance + adjLink[1], path=new_path)
                else:
                    sn = mapping[adjLink[0]]
                    if distance + adjLink[1] < sn.distance:
                        sn.node = v
                        sn.distance = distance + adjLink[1]
                        sn.path = new_path
                heapq.heappush(q, Node(adjLink[0], distance + adjLink[1]))

    return mapping

def find_next_source(V, mapping, visited):  # Receive 'visited' set as an argument
    min_distance = float('inf')
    next_source = None

    for i in range(V):
        if i not in mapping or i in visited:  # Skip visited nodes
            continue
        if mapping[i].distance < min_distance:
            min_distance = mapping[i].distance
            next_source = i

    return next_source

def main():
    adj = [[] for _ in range(6)]
    V = 6
    E = 5
    u = [0, 0, 1, 2, 4]
    v = [3, 5, 4, 5, 5]
    w = [9, 4, 4, 10, 3]

    for i in range(E):
        edge = (v[i], w[i])
        adj[u[i]].append(edge)

        edge2 = (u[i], w[i])
        adj[v[i]].append(edge2)

    source_node = 1
    visited_sources = set()  # Track visited source nodes

    while True:
        mapping = dijkstra(V, adj, source_node)
        visited_sources.add(source_node)  # Mark the source node as visited

        # Find the next source node
        next_source = find_next_source(V, mapping, visited_sources)  # Pass 'visited_sources'

        if next_source is not None:
            print(f"Shortest paths from source node {source_node} to all other nodes:")

            print(f"Next source node: {next_source}")
            source_node = next_source
        else:
            print("No more reachable nodes.")
            break

if __name__ == "__main__":
    main()
