#ifndef BELLMAN_FORD_H
#define BELLMAN_FORD_H

#include "GraphAdjacencyList.hpp"
#include "UnorderedContainers.hpp"

/**
 * @brief A class implementing the Bellman-Ford algorithm for finding the shortest paths from a source vertex to all other vertices in a weighted graph.
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights on the edges.
 */
template <typename Vertex, typename Weight>
class BellmanFord {
public:
  /**
   * @brief Sets the graph to be used for the Bellman-Ford algorithm.
   * @param graph The graph to be used.
   */
  void set_graph(const GraphAdjacencyList<Vertex, Weight>& graph);

  /**
   * @brief Computes the shortest paths from the specified start vertex to all other vertices in the graph.
   * @param start The start vertex.
   */
  void compute(const Vertex& start);

  /**
   * @brief Returns the shortest path from the start vertex to the specified target vertex.
   * @param target The target vertex.
   * @return A pair containing the shortest path and its weight.
   */
  std::pair<std::vector<Vertex>, Weight> shortest_path(const Vertex& target);

  /**
   * @brief Checks if there is a negative weight cycle in the graph.
   * @return A vector of vertices representing a negative weight cycle if one exists, otherwise an empty vector.
   */
  std::vector<Vertex> negative_weight_cycle();

private:
  const GraphAdjacencyList<Vertex, Weight>* graph_;
  UnorderedMap<Vertex, Weight> distance_;
  UnorderedMap<Vertex, Vertex> predecessor_;
  Vertex start_;
};

template <typename Vertex, typename Weight>
void BellmanFord<Vertex, Weight>::set_graph(const GraphAdjacencyList<Vertex, Weight>& graph) {
  graph_ = &graph;
  distance_.clear();
  predecessor_.clear();
}

template <typename Vertex, typename Weight>
void BellmanFord<Vertex, Weight>::compute(const Vertex& start) {
  if (!graph_->has_vertex(start)) return; // Vertex not in graph so do nothing

  start_ = start;
  distance_[start] = 0;
  for (const auto& vertex : graph_->get_vertices()) {
    if (vertex != start) {
      distance_[vertex] = std::numeric_limits<Weight>::max();
    }
  }

  for (int i = 0; i < graph_->get_vertices().size() - 1; ++i) {
    for (const auto& edge : graph_->get_edges()) {
      const Vertex& from = edge.first;
      const Vertex& to = edge.second.first;
      const Weight& weight = edge.second.second;
      if (distance_[from] != std::numeric_limits<Weight>::max() && distance_[from] + weight < distance_[to]) {
        distance_[to] = distance_[from] + weight;
        predecessor_[to] = from;
      }
    }
  }

  for (const auto& edge : graph_->get_edges()) {
    const Vertex& from = edge.first;
    const Vertex& to = edge.second.first;
    const Weight& weight = edge.second.second;
    if (distance_[from] != std::numeric_limits<Weight>::max() && distance_[from] + weight < distance_[to]) {
      throw std::runtime_error("Negative weight cycle detected");
    }
  }
}

template <typename Vertex, typename Weight>
std::pair<std::vector<Vertex>, Weight> BellmanFord<Vertex, Weight>::shortest_path(const Vertex& target) {
  if (distance_.find(target) == distance_.end()) {
    throw std::runtime_error("Target vertex not found in the graph");
  }

  std::vector<Vertex> path;
  Vertex current = target;
  while (current != start_) {
    path.push_back(current);
    current = predecessor_[current];
  }
  path.push_back(start_);
  std::reverse(path.begin(), path.end());

  return std::make_pair(path, distance_[target]);
}

template <typename Vertex, typename Weight>
std::vector<Vertex> BellmanFord<Vertex, Weight>::negative_weight_cycle() {
  UnorderedMap<Vertex, Weight> temp_distance = distance_;
  for (const auto& edge : graph_->get_edges()) {
    const Vertex& from = edge.first;
    const Vertex& to = edge.second.first;
    const Weight& weight = edge.second.second;
    if (temp_distance[from] != std::numeric_limits<Weight>::max() && temp_distance[from] + weight < temp_distance[to]) {
      std::vector<Vertex> cycle;
      Vertex current = to;
      UnorderedSet<Vertex> visited;
      while (true) {
        if (visited.find(current) != visited.end()) {
          // Cycle detected, construct the cycle
          while (current != to) {
            cycle.push_back(current);
            current = predecessor_[current];
          }
          cycle.push_back(to);
          std::reverse(cycle.begin(), cycle.end());
          return cycle;
        }
        visited.insert(current);
        current = predecessor_[current];
      }
    }
  }
  return std::vector<Vertex>();
}

#endif  // BELLMAN_FORD_H

