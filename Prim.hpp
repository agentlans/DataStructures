#ifndef _PRIMSALGORITHM
#define _PRIMSALGORITHM

#include <limits>
#include <unordered_set>
#include <utility>

#include "GraphAdjacencyList.hpp"
#include "MinHeap.hpp"
#include <limits>
#include <unordered_set>
#include <utility>

#include <iostream>

/**
 * @brief Computes the Minimum Spanning Tree (MST) of a given undirected graph using Prim's algorithm.
 *
 * This function takes a pointer to a graph represented as an adjacency list and returns a new graph
 * that represents the Minimum Spanning Tree. The algorithm starts from an arbitrary vertex and 
 * expands the MST by repeatedly adding the minimum weight edge that connects a vertex in the MST 
 * to a vertex outside of it.
 *
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights associated with the edges.
 * @param g A pointer to a GraphAdjacencyList object representing the input graph.
 * @return A GraphAdjacencyList object representing the Minimum Spanning Tree of the input graph.
 *
 * @note This implementation only works for undirected graphs. If the input graph is directed, 
 *       the results may not be valid.
 */
template <class Vertex, class Weight>
GraphAdjacencyList<Vertex, Weight> prim(const GraphAdjacencyList<Vertex, Weight>* g) {
    GraphAdjacencyList<Vertex, Weight> mst;
    if (g->get_vertices().empty()) {
        return mst; // Return empty MST if the graph is empty
    }

    // MinHeap to store vertices based on the minimum edge weight
    MinHeap<Vertex, Weight> min_heap;
    std::unordered_set<Vertex> in_mst; // Track vertices included in MST

    // Start with an arbitrary vertex (the first one in the list)
    Vertex start_vertex = g->get_vertices().front();
    in_mst.insert(start_vertex);
    mst.add_vertex(start_vertex); // Add the starting vertex to the MST

    // Add all edges from the start vertex to the heap
    for (const auto& neighbor : g->get_neighbours_and_weights(start_vertex)) {
        min_heap.set(neighbor.first, neighbor.second);
    }

    while (!min_heap.is_empty()) {
        // Extract the minimum weight edge
        auto [min_vertex, min_weight] = min_heap.extract_min();

        // If the vertex is already in the MST, skip it
        if (in_mst.find(min_vertex) != in_mst.end()) {
            continue;
        }

        // Add the vertex to the MST
        mst.add_vertex(min_vertex); // Add the new vertex to the MST
        mst.add_biedge(start_vertex, min_vertex, min_weight); // Add the edge to the MST
        in_mst.insert(min_vertex);

        // Add all edges from the newly added vertex to the heap
        for (const auto& neighbor : g->get_neighbours_and_weights(min_vertex)) {
            if (in_mst.find(neighbor.first) == in_mst.end()) {
                // Only insert if the vertex is not already in the MST
                // Check if the edge weight is less than the current weight in the heap
                if (min_heap.get(neighbor.first) > neighbor.second) {
                    min_heap.set(neighbor.first, neighbor.second);
                }
            }
        }

        // Update the start vertex for the next iteration
        start_vertex = min_vertex;
    }
    return mst;
}

#endif
