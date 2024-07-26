#include "BreadthFirstSearch.hpp"
#include "GraphAdjacencyList.hpp"
#include <gtest/gtest.h>

class GraphTest : public ::testing::Test {
protected:
  GraphAdjacencyList<char, int> directed_graph;
  GraphAdjacencyList<char, int> undirected_graph;
  BreadthFirstSearch<char, int> bfs;

  void SetUp() override {
    // Set up a directed graph
    directed_graph.add_vertex('A');
    directed_graph.add_vertex('B');
    directed_graph.add_vertex('C');
    directed_graph.add_edge('A', 'B', 1);
    directed_graph.add_edge('B', 'C', 1);
    directed_graph.add_edge('A', 'C', 2); // A -> C (direct edge)

    // Set up an undirected graph
    undirected_graph.add_vertex('A');
    undirected_graph.add_vertex('B');
    undirected_graph.add_vertex('C');
    undirected_graph.add_biedge('A', 'B', 1); // A <-> B
    undirected_graph.add_biedge('B', 'C', 1); // B <-> C
    undirected_graph.add_biedge('C', 'A', 1); // C <-> A
  }
};

TEST_F(GraphTest, BFS_ReachableVertices) {
  bfs.set_graph(directed_graph);
  auto reachable = bfs.bfs('A');
  EXPECT_TRUE(reachable.count('A') > 0);
  EXPECT_TRUE(reachable.count('B') > 0);
  EXPECT_TRUE(reachable.count('C') > 0);
  EXPECT_EQ(reachable.size(), 3); // All vertices should be reachable
}

TEST_F(GraphTest, BFS_NoPath) {
  bfs.set_graph(directed_graph);
  auto reachable = bfs.bfs('C');
  EXPECT_TRUE(reachable.count('C') > 0);
  EXPECT_TRUE(reachable.count('A') == 0);
  EXPECT_TRUE(reachable.count('B') == 0);
}

TEST_F(GraphTest, Path_Exists) {
  bfs.set_graph(directed_graph);
  auto path = bfs.path('A', 'C');
  EXPECT_EQ(path.size(), 2); // A -> C
  EXPECT_EQ(path[0], 'A');
  EXPECT_EQ(path[1], 'C');
}

TEST_F(GraphTest, Path_NoPath) {
  bfs.set_graph(directed_graph);
  auto path = bfs.path('C', 'A');
  EXPECT_TRUE(path.empty()); // No path from C to A in directed graph
}

TEST_F(GraphTest, ConnectedComponents) {
  bfs.set_graph(undirected_graph);
  auto components = bfs.connected_components();
  EXPECT_EQ(components.size(), 1);    // All vertices are connected
  EXPECT_EQ(components[0].size(), 3); // A, B, C are in the same component
}

TEST_F(GraphTest, IsBipartite_True) {
  GraphAdjacencyList<char, int> bipartite_graph;
  bipartite_graph.add_vertex('A');
  bipartite_graph.add_vertex('B');
  bipartite_graph.add_vertex('C');
  bipartite_graph.add_vertex('D');
  bipartite_graph.add_biedge('A', 'B', 1);
  bipartite_graph.add_biedge('A', 'C', 1);
  bipartite_graph.add_biedge('B', 'D', 1);
  bipartite_graph.add_biedge('C', 'D', 1);

  bfs.set_graph(bipartite_graph);
  EXPECT_TRUE(bfs.is_bipartite()); // Should return true
}

TEST_F(GraphTest, IsBipartite_False) {
  GraphAdjacencyList<char, int> non_bipartite_graph;
  non_bipartite_graph.add_vertex('A');
  non_bipartite_graph.add_vertex('B');
  non_bipartite_graph.add_vertex('C');
  non_bipartite_graph.add_biedge('A', 'B', 1);
  non_bipartite_graph.add_biedge('B', 'C', 1);
  non_bipartite_graph.add_biedge('C', 'A', 1); // Creates a cycle of odd length

  bfs.set_graph(non_bipartite_graph);
  EXPECT_FALSE(bfs.is_bipartite()); // Should return false
}

TEST_F(GraphTest, BFS_EmptyGraph) {
  GraphAdjacencyList<char, int> empty_graph;
  bfs.set_graph(empty_graph);
  auto reachable = bfs.bfs('A'); // No vertices to reach
  EXPECT_TRUE(reachable.empty());
}

TEST_F(GraphTest, Path_EmptyGraph) {
  GraphAdjacencyList<char, int> empty_graph;
  bfs.set_graph(empty_graph);
  auto path = bfs.path('A', 'B'); // No path in an empty graph
  EXPECT_TRUE(path.empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
