#include <gtest/gtest.h>
#include "BellmanFord.hpp"

TEST(BellmanFordTest, SimpleGraph) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_edge(0, 1, 2);
  graph.add_edge(1, 2, 3);
  graph.add_edge(2, 0, -5);

  BellmanFord<int, int> bellman_ford;
  bellman_ford.set_graph(graph);
  bellman_ford.compute(0);

  EXPECT_EQ(bellman_ford.shortest_path(1).second, 2);
  EXPECT_EQ(bellman_ford.shortest_path(2).second, 5);
}

TEST(BellmanFordTest, NegativeWeightCycle) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_edge(0, 1, 2);
  graph.add_edge(1, 2, 3);
  graph.add_edge(2, 0, -6);

  BellmanFord<int, int> bellman_ford;
  bellman_ford.set_graph(graph);
  try {
    bellman_ford.compute(0);
  } catch (...) {
  }

  EXPECT_TRUE(bellman_ford.negative_weight_cycle().size() > 0);
}

TEST(BellmanFordTest, DisconnectedGraph) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_edge(0, 1, 2);

  BellmanFord<int, int> bellman_ford;
  bellman_ford.set_graph(graph);
  bellman_ford.compute(0);

  EXPECT_EQ(bellman_ford.shortest_path(1).second, 2);
  EXPECT_EQ(bellman_ford.shortest_path(2).second, std::numeric_limits<int>::max());
}

TEST(BellmanFordTest, SingleVertexGraph) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);

  BellmanFord<int, int> bellman_ford;
  bellman_ford.set_graph(graph);
  bellman_ford.compute(0);

  EXPECT_EQ(bellman_ford.shortest_path(0).second, 0);
}

TEST(BellmanFordTest, EmptyGraph) {
  GraphAdjacencyList<int, int> graph;

  BellmanFord<int, int> bellman_ford;
  bellman_ford.set_graph(graph);
  bellman_ford.compute(0);

  EXPECT_THROW(bellman_ford.shortest_path(0), std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

