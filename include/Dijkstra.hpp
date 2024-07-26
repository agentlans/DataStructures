#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "UnorderedContainers.hpp"
#include "GraphAdjacencyList.hpp"
#include "MinHeap.hpp"

/**
 * @brief A class that implements Dijkstra's algorithm for finding the shortest
 * paths from a starting vertex to all other vertices in a weighted graph.
 *
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights on the edges.
 */
template <class Vertex, class Weight> class Dijkstra {
public:
  /**
   * @brief Sets the graph for the Dijkstra algorithm.
   *
   * @param graph A GraphAdjacencyList representing the graph.
   */
  void set_graph(const GraphAdjacencyList<Vertex, Weight> &graph);

  /**
   * @brief Computes the shortest paths from the specified starting vertex
   *        to all other vertices in the graph.
   *
   * @param start The starting vertex for the shortest path computation.
   * @throws std::invalid_argument If the graph contains negative edge weights.
   */
  void compute(const Vertex &start);

  /**
   * @brief Retrieves the shortest path from the starting vertex to the target
   * vertex.
   *
   * @param target The target vertex for which the shortest path is requested.
   * @return A pair containing a vector of vertices representing the path and
   *         the total weight of the path. If no path exists, the path vector
   *         will be empty and the weight will be set to infinity.
   */
  std::pair<std::vector<Vertex>, Weight> shortest_path(const Vertex &target);

private:
  const GraphAdjacencyList<Vertex, Weight> *graph_ =
      nullptr; ///< Pointer to the graph.
  UnorderedMap<Vertex, Weight>
      distances_; ///< Map of distances from the start vertex.
  UnorderedMap<Vertex, Vertex>
      predecessors_; ///< Map of predecessors for path reconstruction.
  UnorderedMap<Vertex, bool> visited_; ///< Map to track visited vertices.
};

template <class Vertex, class Weight>
void Dijkstra<Vertex, Weight>::set_graph(
    const GraphAdjacencyList<Vertex, Weight> &graph) {
  graph_ = &graph;
}

template <class Vertex, class Weight>
void Dijkstra<Vertex, Weight>::compute(const Vertex &start) {
  MinHeap<Vertex, Weight> min_heap;

  // Initialize distances and predecessors
  for (const auto &vertex : graph_->get_vertices()) {
    distances_[vertex] = std::numeric_limits<Weight>::max();
    visited_[vertex] = false;
    predecessors_[vertex] = Vertex(); // Initialize with a default value
  }

  distances_[start] = 0;
  min_heap.set(start, 0);

  // Main loop of Dijkstra's algorithm
  while (!min_heap.is_empty()) {
    auto [current_vertex, current_distance] = min_heap.extract_min();
    visited_[current_vertex] = true;

    for (const auto &neighbour : graph_->get_neighbours(current_vertex)) {
      if (visited_[neighbour])
        continue;

      Weight edge_weight = graph_->get_weight(current_vertex, neighbour);
      // Check for negative edge weights
      if (edge_weight < 0) {
        throw std::invalid_argument("Graph contains negative edge weights.");
      }

      Weight new_distance = current_distance + edge_weight;

      if (new_distance < distances_[neighbour]) {
        distances_[neighbour] = new_distance;
        predecessors_[neighbour] = current_vertex;
        min_heap.set(neighbour, new_distance);
      }
    }
  }
}

template <class Vertex, class Weight>
std::pair<std::vector<Vertex>, Weight>
Dijkstra<Vertex, Weight>::shortest_path(const Vertex &target) {
  std::vector<Vertex> path;
  Weight total_weight = distances_[target];

  if (total_weight == std::numeric_limits<Weight>::max()) {
    return {path, total_weight}; // No path found
  }

  // Reconstruct the path from target to start
  for (Vertex v = target; v != Vertex(); v = predecessors_[v]) {
    path.push_back(v);
  }

  std::reverse(path.begin(),
               path.end()); // Reverse the path to get the correct order
  return {path, total_weight};
}

#endif // DIJKSTRA_H
