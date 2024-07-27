#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

template <class Vertex, class Weight>
std::pair<GraphAdjacencyList<Vertex, Weight>, Weight>
edmonds_karp(GraphAdjacencyList<Vertex, Weight>& capacity, const Vertex& source, const Vertex& sink) {
    // Create a residual graph initialized with the capacity graph
    GraphAdjacencyList<Vertex, Weight> flow_network;
    for (const auto& vertex : capacity.get_vertices()) {
        flow_network.add_vertex(vertex);
    }
    
    // Initialize flow to zero
    Weight max_flow = 0;

    // BFS to find the augmenting path
    while (true) {
        std::unordered_map<Vertex, Vertex> parent;
        std::unordered_map<Vertex, Weight> capacity_remaining;
        std::queue<Vertex> queue;

        queue.push(source);
        capacity_remaining[source] = std::numeric_limits<Weight>::max();
        parent[source] = source; // Source has no parent

        while (!queue.empty()) {
            Vertex current = queue.front();
            queue.pop();

            // Explore neighbors
            for (const auto& neighbour : capacity.get_neighbours(current)) {
                // Get the total capacity of all edges between current and neighbour
                Weight total_cap = 0;
                for (const auto& edge : capacity.get_neighbours_and_weights(current)) {
                    if (edge.first == neighbour) {
                        total_cap += edge.second; // Sum capacities for multiple edges
                    }
                }

                // Check for valid conditions: not visited and positive capacity
                if (parent.find(neighbour) == parent.end() && total_cap > 0) {
                    parent[neighbour] = current;
                    capacity_remaining[neighbour] = std::min(capacity_remaining[current], total_cap);
                    if (neighbour == sink) {
                        break; // Found a path to the sink
                    }
                    queue.push(neighbour);
                }
            }

            // Break if we found a path to the sink
            if (parent.find(sink) != parent.end()) {
                break;
            }
        }

        // If we didn't reach the sink, we're done
        if (parent.find(sink) == parent.end()) {
            break;
        }

        // Find the maximum flow through the path found
        Weight flow = capacity_remaining[sink];
        max_flow += flow;

        // Update the residual capacities of the edges and reverse edges
        Vertex current = sink;
        while (current != source) {
            Vertex prev = parent[current];
            // Update the flow network (add flow)
            flow_network.add_edge(prev, current, flow);
            // Update the capacity graph (subtract flow)
            for (const auto& edge : capacity.get_neighbours_and_weights(prev)) {
                if (edge.first == current) {
                    capacity.update_edge(prev, current, edge.second - flow);
                    break;
                }
            }
            // Update the reverse edge in the residual graph
            if (capacity.has_edge(current, prev)) {
                capacity.update_edge(current, prev, capacity.get_weight(current, prev) + flow);
            } else {
                capacity.add_edge(current, prev, flow);
            }
            current = prev;
        }
    }

    return {flow_network, max_flow};
}
