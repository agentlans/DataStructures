#ifndef _BFS_H
#define _BFS_H

#include <memory> // For std::unique_ptr
#include <queue>
#include <stdexcept>
#include <vector>

#include "UnorderedContainers.hpp"
#include "GraphAdjacencyList.hpp"

/**
 * @class BreadthFirstSearch
 * @brief A class to perform breadth-first search (BFS) operations on a graph.
 *
 * This class provides methods to explore a graph represented as an adjacency
 * list, allowing users to find reachable vertices, paths between vertices,
 * connected components, and to check if the graph is bipartite.
 *
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights associated with the edges in the
 * graph.
 *
 * Usage:
 * 1. Create an instance of the GraphAdjacencyList class and populate it with
 * vertices and edges.
 * 2. Create an instance of the BreadthFirstSearch class.
 * 3. Use the set_graph method to set the graph for BFS operations.
 * 4. Call the desired BFS methods to perform graph traversal and analysis.
 *
 * Example:
 * @code
 * auto graph = std::make_unique<GraphAdjacencyList<int, float>>();
 * graph->add_vertex(1);
 * graph->add_vertex(2);
 * graph->add_edge(1, 2, 1.0);
 *
 * BreadthFirstSearch<int, float> bfs;
 * bfs.set_graph(graph.get());
 * auto reachable = bfs.bfs(1);
 * auto path = bfs.path(1, 2);
 * auto components = bfs.connected_components();
 * bool isBipartite = bfs.is_bipartite();
 * @endcode
 */
template <class Vertex, class Weight> class BreadthFirstSearch {
public:
  /**
   * Sets the graph to be used for the BFS operations.
   *
   * @param graph The graph to be set for BFS.
   */
  void set_graph(const GraphAdjacencyList<Vertex, Weight> &graph);

  /**
   * Performs a breadth-first search starting from the given vertex.
   *
   * @param start The starting vertex for the BFS.
   * @return A set of vertices reachable from the starting vertex.
   */
  UnorderedSet<Vertex> bfs(const Vertex &start);

  /**
   * Finds the path from the start vertex to the target vertex using BFS.
   *
   * @param start The starting vertex.
   * @param target The target vertex.
   * @return A vector representing the path from start to target, or an empty
   * vector if no path exists.
   */
  std::vector<Vertex> path(const Vertex &start, const Vertex &target);

  // WARNING: connected_components() and is_bipartite() only work for
  // undirected graphs!

  /**
   * Finds all connected components in the graph using BFS.
   *
   * @return A vector of vectors, where each inner vector represents a connected
   * component.
   */
  std::vector<std::vector<Vertex>> connected_components();

  /**
   * Checks if the graph is bipartite using BFS.
   *
   * @return True if the graph is bipartite, false otherwise.
   */
  bool is_bipartite();

private:
  const GraphAdjacencyList<Vertex, Weight> *graph; // Changed to a pointer
};

template <class Vertex, class Weight>
void BreadthFirstSearch<Vertex, Weight>::set_graph(
    const GraphAdjacencyList<Vertex, Weight> &g) {
  graph = &g; // Store the graph for BFS operations
}

template <class Vertex, class Weight>
UnorderedSet<Vertex>
BreadthFirstSearch<Vertex, Weight>::bfs(const Vertex &start) {
  UnorderedSet<Vertex> visited;
  std::queue<Vertex> queue;

  // Starting vertex isn't even in the graph
  if (!graph->has_vertex(start))
    return {};

  queue.push(start);
  visited.insert(start);

  while (!queue.empty()) {
    Vertex current = queue.front();
    queue.pop();

    for (const auto &neighbour : graph->get_neighbours(current)) {
      if (visited.find(neighbour) == visited.end()) {
        visited.insert(neighbour);
        queue.push(neighbour);
      }
    }
  }

  return visited;
}

template <class Vertex, class Weight>
std::vector<Vertex>
BreadthFirstSearch<Vertex, Weight>::path(const Vertex &start,
                                         const Vertex &target) {
  UnorderedMap<Vertex, Vertex> parent;
  UnorderedSet<Vertex> visited;
  std::queue<Vertex> queue;

  queue.push(start);
  visited.insert(start);
  parent[start] = start; // Start has no parent

  while (!queue.empty()) {
    Vertex current = queue.front();
    queue.pop();

    if (current == target) {
      // Reconstruct the path
      std::vector<Vertex> result;
      for (Vertex v = target; v != start; v = parent[v]) {
        result.push_back(v);
      }
      result.push_back(start);
      std::reverse(result.begin(), result.end());
      return result;
    }

    for (const auto &neighbour : graph->get_neighbours(current)) {
      if (visited.find(neighbour) == visited.end()) {
        visited.insert(neighbour);
        queue.push(neighbour);
        parent[neighbour] = current;
      }
    }
  }

  return {}; // Return an empty vector if no path exists
}

template <class Vertex, class Weight>
std::vector<std::vector<Vertex>>
BreadthFirstSearch<Vertex, Weight>::connected_components() {
  UnorderedSet<Vertex> visited; // To keep track of visited vertices
  std::vector<std::vector<Vertex>>
      components; // To store the connected components

  // Get all vertices in the graph
  std::vector<Vertex> vertices = graph->get_vertices();

  // Iterate through each vertex
  for (const Vertex &vertex : vertices) {
    // If the vertex has not been visited, perform BFS
    if (visited.find(vertex) == visited.end()) {
      // Perform BFS to find all reachable vertices from this vertex
      UnorderedSet<Vertex> component = bfs(vertex);

      // Convert the component set to a vector and add it to components
      components.push_back(
          std::vector<Vertex>(component.begin(), component.end()));

      // Mark all vertices in this component as visited
      visited.insert(component.begin(), component.end());
    }
  }

  return components; // Return all connected components
}

template <class Vertex, class Weight>
bool BreadthFirstSearch<Vertex, Weight>::is_bipartite() {
  UnorderedMap<Vertex, int> colour; // 0 or 1 for two colours
  for (const auto &vertex : graph->get_vertices()) {
    if (colour.find(vertex) == colour.end()) {
      std::queue<Vertex> queue;
      queue.push(vertex);
      colour[vertex] = 0; // Start colouring with 0

      while (!queue.empty()) {
        Vertex current = queue.front();
        queue.pop();

        for (const auto &neighbour : graph->get_neighbours(current)) {
          if (colour.find(neighbour) == colour.end()) {
            colour[neighbour] = 1 - colour[current]; // Alternate colour
            queue.push(neighbour);
          } else if (colour[neighbour] == colour[current]) {
            return false; // Same colour found on both ends
          }
        }
      }
    }
  }

  return true; // All vertices can be coloured with two colours
}

#endif
