#ifndef _KRUSKALALGORITHM
#define _KRUSKALALGORITHM

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>

#include "GraphAdjacencyList.hpp"
#include "UnionFind.hpp"
#include "MinHeap.hpp"

/**
 * @brief Computes the Minimum Spanning Tree (MST) of a graph using Kruskal's algorithm.
 *
 * This function takes a pointer to a graph represented as an adjacency list and 
 * constructs its Minimum Spanning Tree (MST) by selecting edges in order of 
 * increasing weight while avoiding cycles using a Union-Find data structure.
 *
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights on the edges.
 * @param g A pointer to a constant GraphAdjacencyList object representing the graph.
 * @return A GraphAdjacencyList object representing the Minimum Spanning Tree of the input graph.
 *
 * @note The input graph must be undirected and connected for the MST to be valid.
 *       The edges are assumed to have non-negative weights.
 */
template <class Vertex, class Weight>
GraphAdjacencyList<Vertex, Weight> kruskal(const GraphAdjacencyList<Vertex, Weight>* g) {

    // Check for negative weights
    for (const auto& vertex : g->get_vertices()) {
        for (const auto& neighbour_weight_pair : g->get_neighbours_and_weights(vertex)) {
            const Weight& weight = neighbour_weight_pair.second;
            if (weight < 0) {
                throw std::invalid_argument("Graph contains negative edge weights.");
            }
        }
    }

    // Create a min-heap to store edges based on their weights
    MinHeap<std::pair<Vertex, Vertex>, Weight> min_heap;

    // Extract all edges from the graph and insert them into the min-heap
    for (const auto& vertex : g->get_vertices()) {
        for (const auto& neighbour_weight_pair : g->get_neighbours_and_weights(vertex)) {
            const Vertex& neighbour = neighbour_weight_pair.first;
            const Weight& weight = neighbour_weight_pair.second;

            // Insert the edge into the min-heap
            if (vertex < neighbour) { // To avoid duplicates (undirected graph)
                min_heap.set(std::make_pair(vertex, neighbour), weight);
            }
        }
    }

    // Create a Union-Find structure to manage connected components
    UnionFind<Vertex> union_find;

    // Add all vertices to the Union-Find structure
    for (const auto& vertex : g->get_vertices()) {
        union_find.insert(vertex);
    }

    // Create a graph to hold the MST
    GraphAdjacencyList<Vertex, Weight> mst;

    // Process edges in order of increasing weight
    while (!min_heap.is_empty()) {
        auto edge = min_heap.extract_min();
        Vertex from = edge.first.first;
        Vertex to = edge.first.second;
        Weight weight = edge.second;

        // Check if the current edge forms a cycle
        if (!union_find.connected(from, to)) {
            // If it doesn't form a cycle, add it to the MST
            mst.add_biedge(from, to, weight);
            // Unite the sets
            union_find.unite(from, to);
        }
    }

    return mst;
}

#endif
