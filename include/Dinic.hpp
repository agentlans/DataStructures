#ifndef _DINIC
#define _DINIC

#include <unordered_map>
#include <queue>
#include <limits>
#include <vector>
#include <utility>
#include <stdexcept>
#include <functional>

#include "GraphAdjacencyList.hpp"

template <class Vertex, class Weight>
std::pair<GraphAdjacencyList<Vertex, Weight>, Weight>
dinic(GraphAdjacencyList<Vertex, Weight>& capacity, const Vertex& source, const Vertex& sink) {
    // Check if the source and sink vertices exist
    if (!capacity.has_vertex(source)) {
        throw std::invalid_argument("Source vertex does not exist in the graph.");
    }
    if (!capacity.has_vertex(sink)) {
        throw std::invalid_argument("Sink vertex does not exist in the graph.");
    }
    
    // Create a flow network initialized to zero
    GraphAdjacencyList<Vertex, Weight> flow_network;
    for (const auto& vertex : capacity.get_vertices()) {
        flow_network.add_vertex(vertex);
    }

    Weight max_flow = 0;

    while (true) {
        // Level graph using BFS
        std::unordered_map<Vertex, int> level;
        std::queue<Vertex> q;
        q.push(source);
        level[source] = 0;

        while (!q.empty()) {
            Vertex u = q.front();
            q.pop();

            for (const auto& [v, cap] : capacity.get_neighbours_and_weights(u)) {
                // Only consider vertices that are not yet leveled and have positive capacity
                if (level.find(v) == level.end() && cap > 0) {
                    level[v] = level[u] + 1;
                    q.push(v);
                }
            }
        }

        // If we can't reach the sink, we are done
        if (level.find(sink) == level.end()) {
            break;
        }

        // Send flow using DFS
        std::function<Weight(Vertex, Weight)> send_flow = [&](Vertex u, Weight flow) {
            if (u == sink) return flow; // If we reached the sink, return the flow
            for (const auto& [v, cap] : capacity.get_neighbours_and_weights(u)) {
                // Check if v is leveled and has positive capacity
                if (level.find(v) != level.end() && level[v] == level[u] + 1 && cap > 0) {
                    Weight current_flow = std::min(flow, cap);
                    Weight temp_flow = send_flow(v, current_flow);
                    if (temp_flow > 0) {
                        // Update capacities in the flow network
                        flow_network.add_edge(u, v, current_flow);
                        flow_network.add_edge(v, u, -current_flow); // Add reverse edge
                        // Update the capacity in the original graph
                        capacity.update_edge(u, v, cap - temp_flow);
                        return temp_flow; // Return the flow sent
                    }
                }
            }
            return Weight(0); // No flow sent
        };

        // Keep sending flow until we can't anymore
        while (true) {
            Weight flow = send_flow(source, std::numeric_limits<Weight>::max());
            if (flow == 0) break; // No more flow can be sent
            max_flow += flow;
        }
    }

    return {flow_network, max_flow};
}

#endif