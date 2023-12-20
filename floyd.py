def find_shortest_path(graph, start, end):
    def find_all_paths(graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return [path]
        if start not in graph:
            return []
        paths = []
        for node in graph[start]:
            if node not in path:
                extended_paths = find_all_paths(graph, node, end, path)
                for p in extended_paths:
                    paths.append(p)
        return paths

    def find_shortest(paths):
        if not paths:
            return None
        shortest = paths[0]
        for path in paths:
            if len(path) < len(shortest):
                shortest = path
        return shortest

    all_paths = find_all_paths(graph, start, end)
    if not all_paths:
        print(f"There is no path from {start} to {end}.")
        return None

    shortest_path = find_shortest(all_paths)
    print(f"The shortest path from {start} to {end} is: {shortest_path}")
    return shortest_path

# Example graph
graph = {
    0: [1, 2],
    1: [3],
    2: [4],
    3: [4],
    4: []
}

# Example usage
start_node = 0
end_node = 4
shortest_path = find_shortest_path(graph, start_node, end_node)
