#ifndef _DEPTHFIRSTSEARCH
#define _DEPTHFIRSTSEARCH

#include <algorithm>
#include <stack>
#include <stdexcept>
#include <vector>

#include "UnorderedContainers.hpp"
#include "GraphAdjacencyList.hpp"

/**
 * @brief A class to perform depth-first search on a graph.
 *
 * This class provides methods to traverse the graph, find paths, detect cycles,
 * and perform topological sorting. It also includes functionality to check if
 * the graph is bipartite.
 *
 * @tparam Vertex The type of the vertices in the graph.
 * @tparam Weight The type of the weights on the edges.
 */
template <class Vertex, class Weight> class DepthFirstSearch {
private:
  const GraphAdjacencyList<Vertex, Weight> *graph;

public:
  DepthFirstSearch() : graph(nullptr) {}

  /**
   * @brief Sets the graph for the depth-first search.
   *
   * @param graph The graph to be used for the search.
   */
  void set_graph(const GraphAdjacencyList<Vertex, Weight> &graph);

  /**
   * @brief Performs depth-first search starting from a given vertex.
   *
   * @param start The starting vertex for the search.
   * @return A set of vertices visited during the search.
   */
  UnorderedSet<Vertex> dfs(const Vertex &start);

  /**
   * @brief Finds a path from the start vertex to the target vertex.
   *
   * @param start The starting vertex.
   * @param target The target vertex.
   * @return A vector representing the path from start to target.
   */
  std::vector<Vertex> path(const Vertex &start, const Vertex &target);

  /**
   * @brief Finds a cycle in the graph, if one exists.
   *
   * @return A vector of vertices representing the cycle, or an empty vector if
   * no cycle exists.
   */
  std::vector<Vertex> find_cycle();

  /**
   * @brief Performs a topological sort of the graph.
   *
   * @return A vector of vertices in topologically sorted order.
   * @throws std::runtime_error if the graph contains a cycle.
   */
  std::vector<Vertex> topological_sort();

  // WARNING: is_bipartite() is only to be used for undirected graphs!

  /**
   * @brief Checks if the graph is bipartite.
   *
   * @return True if the graph is bipartite, false otherwise.
   */
  bool is_bipartite();
};

template <class Vertex, class Weight>
void DepthFirstSearch<Vertex, Weight>::set_graph(
    const GraphAdjacencyList<Vertex, Weight> &g) {
  graph = &g;
}

template <class Vertex, class Weight>
UnorderedSet<Vertex>
DepthFirstSearch<Vertex, Weight>::dfs(const Vertex &start) {
  UnorderedSet<Vertex> visited;
  std::stack<Vertex> stack;

  // Starting vertex isn't even in the graph
  if (!graph->has_vertex(start))
    return {};

  stack.push(start);

  while (!stack.empty()) {
    Vertex vertex = stack.top();
    stack.pop();

    if (visited.find(vertex) == visited.end()) {
      visited.insert(vertex);
      for (const auto &neighbour : graph->get_neighbours(vertex)) {
        stack.push(neighbour);
      }
    }
  }
  return visited;
}

template <class Vertex, class Weight>
std::vector<Vertex>
DepthFirstSearch<Vertex, Weight>::path(const Vertex &start,
                                       const Vertex &target) {
  UnorderedSet<Vertex> visited;
  std::vector<Vertex> result;
  std::stack<std::pair<Vertex, std::vector<Vertex>>> stack;
  stack.push({start, {start}});

  while (!stack.empty()) {
    auto [vertex, current_path] = stack.top();
    stack.pop();

    if (vertex == target) {
      return current_path;
    }

    if (visited.find(vertex) == visited.end()) {
      visited.insert(vertex);
      for (const auto &neighbour : graph->get_neighbours(vertex)) {
        std::vector<Vertex> new_path = current_path;
        new_path.push_back(neighbour);
        stack.push({neighbour, new_path});
      }
    }
  }
  return {}; // Return empty vector if no path found
}

template <class Vertex, class Weight>
std::vector<Vertex> DepthFirstSearch<Vertex, Weight>::find_cycle() {
  UnorderedSet<Vertex> visited;
  UnorderedSet<Vertex> rec_stack;
  std::vector<Vertex> cycle;

  std::function<bool(Vertex)> dfs_visit = [&](Vertex vertex) {
    visited.insert(vertex);
    rec_stack.insert(vertex);

    for (const auto &neighbour : graph->get_neighbours(vertex)) {
      if (rec_stack.find(neighbour) != rec_stack.end()) {
        cycle.push_back(neighbour); // Cycle detected
        return true;
      }
      if (visited.find(neighbour) == visited.end()) {
        if (dfs_visit(neighbour)) {
          if (!cycle.empty() && cycle.back() != neighbour) {
            cycle.push_back(neighbour);
          }
          return true;
        }
      }
    }
    rec_stack.erase(vertex);
    return false;
  };

  for (const auto &vertex : graph->get_vertices()) {
    if (visited.find(vertex) == visited.end()) {
      if (dfs_visit(vertex)) {
        std::reverse(cycle.begin(), cycle.end());
        return cycle;
      }
    }
  }
  return {}; // Return empty vector if no cycle found
}

template <class Vertex, class Weight>
std::vector<Vertex> DepthFirstSearch<Vertex, Weight>::topological_sort() {
  UnorderedSet<Vertex> visited;
  UnorderedSet<Vertex> rec_stack; // To track vertices in the current path
  std::stack<Vertex> stack;
  std::vector<Vertex> result;

  std::function<bool(Vertex)> topological_visit = [&](Vertex vertex) {
    if (rec_stack.find(vertex) != rec_stack.end()) {
      return true; // Cycle detected
    }
    if (visited.find(vertex) != visited.end()) {
      return false; // Already visited
    }

    visited.insert(vertex);
    rec_stack.insert(vertex); // Add to recursion stack

    for (const auto &neighbour : graph->get_neighbours(vertex)) {
      if (topological_visit(neighbour)) {
        return true; // Cycle detected in recursion
      }
    }

    rec_stack.erase(vertex); // Remove from recursion stack
    stack.push(vertex);
    return false; // No cycle detected
  };

  for (const auto &vertex : graph->get_vertices()) {
    if (visited.find(vertex) == visited.end()) {
      if (topological_visit(vertex)) {
        throw std::runtime_error(
            "Graph contains a cycle, topological sort not possible.");
      }
    }
  }

  while (!stack.empty()) {
    result.push_back(stack.top());
    stack.pop();
  }

  return result;
}

template <class Vertex, class Weight>
bool DepthFirstSearch<Vertex, Weight>::is_bipartite() {
  UnorderedMap<Vertex, int> colour;
  for (const auto &vertex : graph->get_vertices()) {
    if (colour.find(vertex) == colour.end()) {
      std::stack<Vertex> stack;
      stack.push(vertex);
      colour[vertex] = 0; // Start colouring with 0

      while (!stack.empty()) {
        Vertex current = stack.top();
        stack.pop();

        for (const auto &neighbour : graph->get_neighbours(current)) {
          if (colour.find(neighbour) == colour.end()) {
            colour[neighbour] = 1 - colour[current]; // Alternate colour
            stack.push(neighbour);
          } else if (colour[neighbour] == colour[current]) {
            return false; // Same colour found on both sides
          }
        }
      }
    }
  }
  return true; // No conflicts found, graph is bipartite
}

#endif