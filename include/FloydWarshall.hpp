#ifndef _FLOYDWARSHALL
#define _FLOYDWARSHALL

#include <stdexcept>
#include "GraphAdjacencyList.hpp"

/**
 * @brief A class implementing the Floyd-Warshall algorithm for finding the shortest paths in a weighted graph.
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights on the edges.
 */
template <typename Vertex, typename Weight>
class FloydWarshall {
public:
  /**
   * @brief Sets the graph to be used for the Floyd-Warshall algorithm.
   * @param graph The graph to be used.
   */
  void set_graph(const GraphAdjacencyList<Vertex, Weight>& graph);

  /**
   * @brief Computes the shortest paths between all pairs of vertices.
   */
  void compute();

  /**
   * @brief Finds the shortest path from the source vertex to the target vertex.
   * @param source The source vertex.
   * @param target The target vertex.
   * @return A pair containing the shortest path and its weight.
   */
  std::pair<std::vector<Vertex>, Weight> shortest_path(const Vertex& source, const Vertex& target);

private:
  /**
   * @brief A pointer to the graph being used.
   */
  const GraphAdjacencyList<Vertex, Weight>* graph_;
  std::vector<std::vector<Weight>> distance_matrix_;
  std::vector<std::vector<Vertex>> predecessor_matrix_;
};

template <typename Vertex, typename Weight>
void FloydWarshall<Vertex, Weight>::set_graph(const GraphAdjacencyList<Vertex, Weight>& graph) {
  graph_ = &graph;
}

template <typename Vertex, typename Weight>
void FloydWarshall<Vertex, Weight>::compute() {
  std::vector<Vertex> vertices = graph_->get_vertices();
  size_t num_vertices = vertices.size();

  distance_matrix_.resize(num_vertices, std::vector<Weight>(num_vertices));
  predecessor_matrix_.resize(num_vertices, std::vector<Vertex>(num_vertices));

  // Initialize the distance matrix and predecessor matrix
  for (size_t i = 0; i < num_vertices; ++i) {
    for (size_t j = 0; j < num_vertices; ++j) {
      if (i == j) {
        distance_matrix_[i][j] = 0;
      } else {
        distance_matrix_[i][j] = std::numeric_limits<Weight>::max();
      }
      predecessor_matrix_[i][j] = vertices[i];
    }
  }

  // Initialize the distance matrix with the weights from the graph
  for (size_t i = 0; i < num_vertices; ++i) {
    for (size_t j = 0; j < num_vertices; ++j) {
      if (graph_->has_edge(vertices[i], vertices[j])) {
        distance_matrix_[i][j] = graph_->get_weight(vertices[i], vertices[j]);
      }
    }
  }

  // Compute the shortest paths
  for (size_t k = 0; k < num_vertices; ++k) {
    for (size_t i = 0; i < num_vertices; ++i) {
      for (size_t j = 0; j < num_vertices; ++j) {
        if (distance_matrix_[i][k] != std::numeric_limits<Weight>::max() &&
            distance_matrix_[k][j] != std::numeric_limits<Weight>::max() &&
            distance_matrix_[i][k] + distance_matrix_[k][j] < distance_matrix_[i][j]) {
          distance_matrix_[i][j] = distance_matrix_[i][k] + distance_matrix_[k][j];
          predecessor_matrix_[i][j] = predecessor_matrix_[k][j];
        }
      }
    }
  }
}

template <typename Vertex, typename Weight>
std::pair<std::vector<Vertex>, Weight> FloydWarshall<Vertex, Weight>::shortest_path(const Vertex& source, const Vertex& target) {
  if (!graph_->has_vertex(source) || !graph_->has_vertex(target)) {
    throw std::invalid_argument("Source or target vertex not in the graph.");
  }

  std::vector<Vertex> vertices = graph_->get_vertices();
  size_t source_index = std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(), source));
  size_t target_index = std::distance(vertices.begin(), std::find(vertices.begin(), vertices.end(), target));

  if (distance_matrix_[source_index][target_index] == std::numeric_limits<Weight>::max()) {
    return std::make_pair(std::vector<Vertex>(), std::numeric_limits<Weight>::max());
  }

  std::vector<Vertex> path;
  Vertex current = target;
  while (current != source) {
    path.push_back(current);
    if (current == predecessor_matrix_[source_index][target_index]) {
      break;
    }
    current = predecessor_matrix_[source_index][target_index];
  }
  path.push_back(source);
  std::reverse(path.begin(), path.end());

  return std::make_pair(path, distance_matrix_[source_index][target_index]);
}


#endif
