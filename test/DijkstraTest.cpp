#include <gtest/gtest.h>

#include "Dijkstra.hpp"
#include "GraphAdjacencyList.hpp"

class DijkstraTest : public ::testing::Test {
protected:
  Dijkstra<int, double> dijkstra; // Use appropriate types for Vertex and Weight
  GraphAdjacencyList<int, double> graph;

  void SetUp() override {
    // Example graph setup
    graph.add_edge(1, 2, 1.0);
    graph.add_edge(1, 3, 4.0);
    graph.add_edge(2, 3, 2.0);
    graph.add_edge(3, 4, 1.0);
    graph.add_edge(2, 4, 5.0);
  }
};

TEST_F(DijkstraTest, ShortestPathFromStart) {
  dijkstra.set_graph(graph);
  dijkstra.compute(1);

  auto [path, weight] = dijkstra.shortest_path(4);

  EXPECT_EQ(path.size(), 4); // Path should contain 4 vertices: 1 -> 2 -> 3 -> 4
  EXPECT_EQ(path[0], 1);
  EXPECT_EQ(path[1], 2);
  EXPECT_EQ(path[2], 3);
  EXPECT_DOUBLE_EQ(weight, 4.0); // Total weight should be 4.0
}

TEST_F(DijkstraTest, NoPathExists) {
  graph.add_edge(5, 6, 1.0); // Add an isolated edge
  dijkstra.set_graph(graph);
  dijkstra.compute(1);

  auto [path, weight] = dijkstra.shortest_path(5);

  EXPECT_TRUE(path.empty()); // No path should exist
  EXPECT_DOUBLE_EQ(
      weight,
      std::numeric_limits<double>::infinity()); // Weight should be infinity
}

TEST_F(DijkstraTest, SingleVertexGraph) {
  GraphAdjacencyList<int, double> single_vertex_graph;
  single_vertex_graph.add_vertex(1);

  dijkstra.set_graph(single_vertex_graph);
  dijkstra.compute(1);

  auto [path, weight] = dijkstra.shortest_path(1);

  EXPECT_EQ(path.size(), 1); // Path should contain only the starting vertex
  EXPECT_EQ(path[0], 1);
  EXPECT_DOUBLE_EQ(weight, 0.0); // Weight should be 0.0
}

TEST_F(DijkstraTest, GraphWithNegativeWeights) {
  GraphAdjacencyList<int, double> graph;
  graph.add_edge(1, 2, -1.0); // Add a negative weight edge
  dijkstra.set_graph(graph);

  // Since Dijkstra's algorithm does not handle negative weights, we can expect
  // an error or incorrect behavior
  EXPECT_THROW(
      dijkstra.compute(1),
      std::invalid_argument); // Assuming compute throws on negative weights
}

TEST_F(DijkstraTest, DisconnectedGraph) {
  GraphAdjacencyList<int, double> disconnected_graph;
  disconnected_graph.add_edge(1, 2, 1.0);
  disconnected_graph.add_edge(3, 4,
                              2.0); // Completely disconnected from 1 and 2

  dijkstra.set_graph(disconnected_graph);
  dijkstra.compute(1);

  auto [path, weight] = dijkstra.shortest_path(3);

  EXPECT_TRUE(path.empty()); // No path should exist
  EXPECT_DOUBLE_EQ(
      weight,
      std::numeric_limits<double>::infinity()); // Weight should be infinity
}

TEST_F(DijkstraTest, MultipleShortestPaths) {
  GraphAdjacencyList<int, double> graph;
  graph.add_edge(1, 2, 1.0);
  graph.add_edge(1, 3, 1.0);
  graph.add_edge(2, 4, 1.0);
  graph.add_edge(3, 4, 1.0); // Two paths: 1->2->4 and 1->3->4
  dijkstra.set_graph(graph);
  dijkstra.compute(1);

  auto [path, weight] = dijkstra.shortest_path(4);
  std::sort(path.begin(), path.end());

  EXPECT_TRUE((path == std::vector<int>{1, 2, 4} ||
               path == std::vector<int>{1, 3, 4})); // Either path is valid
  EXPECT_DOUBLE_EQ(weight, 2.0); // Total weight should be 2.0
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
