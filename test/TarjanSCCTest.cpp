#include "TarjanSCC.hpp"
#include "GraphAdjacencyList.hpp"
#include <gtest/gtest.h>

// Test fixture for Tarjan's SCC tests
class TarjanSCCTest : public ::testing::Test {
protected:
  GraphAdjacencyList<int, int> graph;

  void SetUp() override {
    // You can set up common test data here if needed
  }

  void TearDown() override {
    // Clean up after each test if necessary
  }
};

// Test case for an empty graph
TEST_F(TarjanSCCTest, EmptyGraph) {
  auto result = tarjan_scc(graph);
  EXPECT_TRUE(result.empty());
}

// Test case for a single vertex
TEST_F(TarjanSCCTest, SingleVertex) {
  graph.add_vertex(1);
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].size(), 1);
  EXPECT_EQ(result[0][0], 1);
}

// Test case for two vertices with no edges
TEST_F(TarjanSCCTest, TwoVerticesNoEdges) {
  graph.add_vertex(1);
  graph.add_vertex(2);
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0].size(), 1);
  EXPECT_EQ(result[1].size(), 1);
  int values[] = {result[0][0], result[1][0]};
  std::sort(values, values + 2);
  EXPECT_EQ(values[0], 1);
  EXPECT_EQ(values[1], 2);
}

// Test case for two vertices with a single edge
TEST_F(TarjanSCCTest, TwoVerticesOneEdge) {
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_edge(1, 2, 1);
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0].size(), 1);
  EXPECT_EQ(result[1].size(), 1);
  int values[] = {result[0][0], result[1][0]};
  std::sort(values, values + 2);
  EXPECT_EQ(values[0], 1);
  EXPECT_EQ(values[1], 2);
}

// Test case for a cycle
TEST_F(TarjanSCCTest, Cycle) {
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_vertex(3);
  graph.add_edge(1, 2, 1);
  graph.add_edge(2, 3, 1);
  graph.add_edge(3, 1, 1);
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].size(), 3);
  EXPECT_TRUE(
      (std::find(result[0].begin(), result[0].end(), 1) != result[0].end()));
  EXPECT_TRUE(
      (std::find(result[0].begin(), result[0].end(), 2) != result[0].end()));
  EXPECT_TRUE(
      (std::find(result[0].begin(), result[0].end(), 3) != result[0].end()));
}

// Test case for multiple strongly connected components
TEST_F(TarjanSCCTest, MultipleSCCs) {
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_vertex(3);
  graph.add_vertex(4);
  graph.add_edge(1, 2, 1);
  graph.add_edge(2, 1, 1); // SCC 1
  graph.add_edge(3, 4, 1);
  graph.add_edge(4, 3, 1); // SCC 2
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 2);
  EXPECT_EQ(result[0].size(), 2); // SCC with vertices 1 and 2
  EXPECT_EQ(result[1].size(), 2); // SCC with vertices 3 and 4
}

// Test case for a disconnected graph
TEST_F(TarjanSCCTest, DisconnectedGraph) {
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_vertex(3);
  graph.add_vertex(4);
  graph.add_edge(1, 2, 1);
  graph.add_edge(3, 4, 1); // Two separate components
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 4); // Each vertex should be its own SCC
}

// Test case for self-loop
TEST_F(TarjanSCCTest, SelfLoop) {
  graph.add_vertex(1);
  graph.add_edge(1, 1, 1); // Self-loop
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0].size(), 1);
  EXPECT_EQ(result[0][0], 1);
}

// Test case for complex graph
TEST_F(TarjanSCCTest, ComplexGraph) {
  // GraphAdjacencyList<int, int> graph;
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_vertex(3);
  graph.add_vertex(4);
  graph.add_vertex(5);
  graph.add_edge(1, 2, 1);
  graph.add_edge(2, 3, 1);
  graph.add_edge(3, 1, 1); // SCC 1
  graph.add_edge(4, 5, 1);
  graph.add_edge(5, 4, 1); // SCC 2
  graph.add_edge(2, 4, 1); // Edge to another component
  auto result = tarjan_scc(graph);
  EXPECT_EQ(result.size(), 2); // Two SCCs (4, 5) and (1, 2, 3)
  size_t sizes[] = {result[0].size(), result[1].size()};
  std::sort(sizes, sizes + 2);
  EXPECT_EQ(sizes[0], 2);
  EXPECT_EQ(sizes[1], 3);
}

// Main function to run all tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
