class Graph:
    def __init__(self):
        self.edges = []

    def add_edge(self, u, v, weight):
        self.edges.append((u, v, weight))


def bellman_ford(graph, start, end):
    def initialize_single_source(graph, start):
        distance = {node: float('inf') for node in range(len(graph.edges))}
        distance[start] = 0
        return distance

    def relax(edge, distance):
        u, v, weight = edge
        if distance[u] + weight < distance[v]:
            distance[v] = distance[u] + weight

    def find_shortest_path(graph, start, end):
        distance = initialize_single_source(graph, start)

        for _ in range(len(graph.edges) - 1):
            for edge in graph.edges:
                relax(edge, distance)

        for edge in graph.edges:
            u, v, weight = edge
            if distance[u] + weight < distance[v]:
                print("Graph contains negative weight cycle. Cannot find the shortest path.")
                return None

        return distance

    # Example usage
    start_node = start  # Replace with your actual start node
    end_node = end      # Replace with your actual end node
    shortest_distances = find_shortest_path(graph, start_node, end_node)

    if shortest_distances:
        print(f"The shortest distance from {start_node} to {end_node} is: {shortest_distances[end_node]}")


# Example graph
graph = Graph()
graph.add_edge(0, 1, 1)
graph.add_edge(0, 2, 3)
graph.add_edge(1, 3, 2)
graph.add_edge(2, 4, 1)
graph.add_edge(3, 4, 3)

# Replace start and end with your actual start and end nodes
start_node = 0
end_node = 4
bellman_ford(graph, start_node, end_node)
