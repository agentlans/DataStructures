#ifndef _TARJANSCC
#define _TARJANSCC

#include <algorithm>
#include <stack>
#include <vector>

#include "UnorderedContainers.hpp"
#include "GraphAdjacencyList.hpp"

/**
 * @brief Computes the strongly connected components (SCCs) of a directed graph.
 *
 * This function uses Tarjan's algorithm to find all strongly connected
 * components in the given graph. A strongly connected component is a maximal
 * subgraph where every vertex is reachable from every other vertex in that
 * subgraph.
 *
 * @param g The graph represented as an adjacency list.
 * @return A vector of vectors, where each inner vector contains the vertices
 *         of one strongly connected component.
 */
template <typename Vertex, typename Weight>
std::vector<std::vector<Vertex>>
tarjan_scc(const GraphAdjacencyList<Vertex, Weight> &g);

/**
 * @brief Computes the strongly connected components (SCCs) of a directed graph.
 *
 * This function implements Tarjan's algorithm to find all strongly connected
 * components in the given graph. It maintains a depth-first search (DFS) and
 * uses indices and low-link values to identify SCCs.
 *
 * @param g The graph represented as an adjacency list.
 * @return A vector of vectors, where each inner vector contains the vertices
 *         of one strongly connected component.
 */
template <typename Vertex, typename Weight>
std::vector<std::vector<Vertex>>
tarjan_scc(const GraphAdjacencyList<Vertex, Weight> &g) {
  UnorderedMap<Vertex, int> index_map;
  UnorderedMap<Vertex, int> low_link_map;
  std::stack<Vertex> stack;
  std::vector<std::vector<Vertex>> strongly_connected_components;
  std::vector<Vertex> on_stack;
  int index = 0;

  std::function<void(const Vertex &)> strongconnect = [&](const Vertex &v) {
    index_map[v] = index;
    low_link_map[v] = index;
    index++;
    stack.push(v);
    on_stack.push_back(v);

    for (const auto &neighbor : g.get_neighbours(v)) {
      if (index_map.find(neighbor) == index_map.end()) {
        // If neighbor has not yet been visited, recurse on it
        strongconnect(neighbor);
        low_link_map[v] = std::min(low_link_map[v], low_link_map[neighbor]);
      } else if (std::find(on_stack.begin(), on_stack.end(), neighbor) !=
                 on_stack.end()) {
        // If neighbor is in the stack, it's part of the current SCC
        low_link_map[v] = std::min(low_link_map[v], index_map[neighbor]);
      }
    }

    // If v is a root node, pop the stack and generate an SCC
    if (low_link_map[v] == index_map[v]) {
      std::vector<Vertex> component;
      Vertex w;
      do {
        w = stack.top();
        stack.pop();
        on_stack.pop_back();
        component.push_back(w);
        low_link_map[w] = index_map[v]; // Assign the root's low link value
      } while (w != v);
      strongly_connected_components.push_back(component);
    }
  };

  for (const auto &vertex : g.get_vertices()) {
    if (index_map.find(vertex) == index_map.end()) {
      strongconnect(vertex);
    }
  }

  return strongly_connected_components;
}

#endif
