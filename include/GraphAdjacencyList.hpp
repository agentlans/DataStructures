#ifndef GRAPH_H
#define GRAPH_H

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * @brief A class representing a graph using an adjacency list.
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights on the edges.
 */
template <typename Vertex, typename Weight> class GraphAdjacencyList {
private:
  std::unordered_map<Vertex, std::vector<std::pair<Vertex, Weight>>>
      adjacency_list;

public:
  /**
   * @brief Add a vertex to the graph.
   * @param vertex The vertex to be added.
   */
  void add_vertex(const Vertex &vertex);

  /**
   * @brief Update a vertex in the graph.
   * @param old_vertex The vertex to be updated.
   * @param new_vertex The new vertex value.
   * @throw std::runtime_error if the old vertex does not exist.
   */
  void update_vertex(const Vertex &old_vertex, const Vertex &new_vertex);

  /**
   * @brief Remove a vertex from the graph.
   * @param vertex The vertex to be removed.
   * @throw std::runtime_error if the vertex does not exist.
   */
  void remove_vertex(const Vertex &vertex);

  /**
   * @brief Check if a vertex exists in the graph.
   * @param vertex The vertex to check.
   * @return true if the vertex exists, false otherwise.
   */
  bool has_vertex(const Vertex &vertex) const;

  /**
   * @brief Add an edge to the graph.
   * @param from The source vertex.
   * @param to The destination vertex.
   * @param weight The weight of the edge.
   */
  void add_edge(const Vertex &from, const Vertex &to, const Weight &weight);

  /**
   * @brief Add an edge to the graph in both directions.
   * @param from The source vertex.
   * @param to The destination vertex.
   * @param weight The weight of the edge.
   */
  void add_biedge(const Vertex &from, const Vertex &to, const Weight &weight);

  /**
   * @brief Update the weight of an edge.
   * @param from The source vertex.
   * @param to The destination vertex.
   * @param new_weight The new weight for the edge.
   * @throw std::runtime_error if the edge does not exist.
   */
  void update_edge(const Vertex &from, const Vertex &to,
                   const Weight &new_weight);

  /**
   * @brief Remove an edge from the graph.
   * @param from The source vertex.
   * @param to The destination vertex.
   */
  void remove_edge(const Vertex &from, const Vertex &to);

  /**
   * @brief Check if an edge exists between two vertices.
   * @param from The source vertex.
   * @param to The destination vertex.
   * @return true if the edge exists, false otherwise.
   */
  bool has_edge(const Vertex &from, const Vertex &to) const;

  /**
   * @brief Get the weight of an edge.
   * @param from The source vertex.
   * @param to The destination vertex.
   * @return The weight of the edge.
   * @throw std::runtime_error if the edge does not exist.
   */
  Weight get_weight(const Vertex &from, const Vertex &to) const;

  /**
   * @brief Get all neighbours of a vertex.
   * @param vertex The vertex to get neighbours for.
   * @return A vector of neighbouring vertices.
   */
  std::vector<Vertex> get_neighbours(const Vertex &vertex) const;

  /**
   * @brief Get all neighbours of a vertex with their weights.
   * @param vertex The vertex to get neighbours for.
   * @return A vector of pairs, each containing a neighbouring vertex and the
   * edge weight.
   */
  std::vector<std::pair<Vertex, Weight>>
  get_neighbours_and_weights(const Vertex &vertex) const;

  /**
   * @brief Get all vertices in the graph.
   * @return A vector of all vertices in the graph.
   */
  std::vector<Vertex> get_vertices() const;

  /**
   * @brief Print the graph structure.
   */
  void print_graph() const;
};

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::add_vertex(const Vertex &vertex) {
  if (adjacency_list.find(vertex) == adjacency_list.end()) {
    adjacency_list[vertex] = std::vector<std::pair<Vertex, Weight>>();
  }
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::add_edge(const Vertex &from,
                                                  const Vertex &to,
                                                  const Weight &weight) {
  add_vertex(from);
  add_vertex(to);
  adjacency_list[from].push_back(std::make_pair(to, weight));
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::add_biedge(const Vertex &from,
                                                    const Vertex &to,
                                                    const Weight &weight) {
  add_edge(from, to, weight);
  add_edge(to, from, weight);
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::remove_edge(const Vertex &from,
                                                     const Vertex &to) {
  if (adjacency_list.find(from) != adjacency_list.end()) {
    auto &edges = adjacency_list[from];
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [&to](const std::pair<Vertex, Weight> &edge) {
                                 return edge.first == to;
                               }),
                edges.end());
  }
}

template <typename Vertex, typename Weight>
bool GraphAdjacencyList<Vertex, Weight>::has_edge(const Vertex &from,
                                                  const Vertex &to) const {
  if (adjacency_list.find(from) != adjacency_list.end()) {
    const auto &edges = adjacency_list.at(from);
    return std::find_if(edges.begin(), edges.end(),
                        [&to](const std::pair<Vertex, Weight> &edge) {
                          return edge.first == to;
                        }) != edges.end();
  }
  return false;
}

template <typename Vertex, typename Weight>
Weight GraphAdjacencyList<Vertex, Weight>::get_weight(const Vertex &from,
                                                      const Vertex &to) const {
  if (adjacency_list.find(from) != adjacency_list.end()) {
    const auto &edges = adjacency_list.at(from);
    auto it = std::find_if(edges.begin(), edges.end(),
                           [&to](const std::pair<Vertex, Weight> &edge) {
                             return edge.first == to;
                           });
    if (it != edges.end()) {
      return it->second;
    }
  }
  throw std::runtime_error("Edge not found");
}

template <typename Vertex, typename Weight>
std::vector<Vertex>
GraphAdjacencyList<Vertex, Weight>::get_neighbours(const Vertex &vertex) const {
  std::vector<Vertex> neighbours;
  if (adjacency_list.find(vertex) != adjacency_list.end()) {
    const auto &edges = adjacency_list.at(vertex);
    neighbours.reserve(edges.size());
    for (const auto &edge : edges) {
      neighbours.push_back(edge.first);
    }
  }
  return neighbours;
}

template <typename Vertex, typename Weight>
std::vector<std::pair<Vertex, Weight>>
GraphAdjacencyList<Vertex, Weight>::get_neighbours_and_weights(
    const Vertex &vertex) const {
  auto it = adjacency_list.find(vertex);
  if (it != adjacency_list.end()) {
    return it->second;
  }
  return {}; // Return an empty vector if the vertex is not found
}

template <typename Vertex, typename Weight>
std::vector<Vertex> GraphAdjacencyList<Vertex, Weight>::get_vertices() const {
  std::vector<Vertex> vertices;
  vertices.reserve(adjacency_list.size());
  for (const auto &pair : adjacency_list) {
    vertices.push_back(pair.first);
  }
  return vertices;
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::print_graph() const {
  for (const auto &entry : adjacency_list) {
    std::cout << entry.first << ": ";
    for (const auto &edge : entry.second) {
      std::cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    std::cout << std::endl;
  }
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::update_vertex(
    const Vertex &old_vertex, const Vertex &new_vertex) {
  if (adjacency_list.find(old_vertex) == adjacency_list.end()) {
    throw std::runtime_error("Vertex does not exist.");
  }
  // Move edges from old_vertex to new_vertex
  adjacency_list[new_vertex] = std::move(adjacency_list[old_vertex]);
  adjacency_list.erase(old_vertex);

  // Update edges pointing to old_vertex
  for (auto &pair : adjacency_list) {
    auto &edges = pair.second;
    for (auto &edge : edges) {
      if (edge.first == old_vertex) {
        edge.first = new_vertex; // Update destination vertex
      }
    }
  }
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::remove_vertex(const Vertex &vertex) {
  if (adjacency_list.find(vertex) == adjacency_list.end()) {
    throw std::runtime_error("Vertex does not exist.");
  }
  // Remove all edges associated with the vertex
  adjacency_list.erase(vertex);
  for (auto &pair : adjacency_list) {
    auto &edges = pair.second;
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [&](const std::pair<Vertex, Weight> &edge) {
                                 return edge.first == vertex;
                               }),
                edges.end());
  }
}

template <typename Vertex, typename Weight>
void GraphAdjacencyList<Vertex, Weight>::update_edge(const Vertex &from,
                                                     const Vertex &to,
                                                     const Weight &new_weight) {
  if (!has_edge(from, to)) {
    throw std::runtime_error("Edge does not exist.");
  }
  for (auto &edge : adjacency_list[from]) {
    if (edge.first == to) {
      edge.second = new_weight; // Update the weight of the edge
      break;
    }
  }
}

template <typename Vertex, typename Weight>
bool GraphAdjacencyList<Vertex, Weight>::has_vertex(
    const Vertex &vertex) const {
  return adjacency_list.find(vertex) != adjacency_list.end();
}

#endif // GRAPH_H
