#include <gtest/gtest.h>

#include "GraphAdjacencyList.hpp"
#include "FloydWarshall.hpp"
#include "CompareElements.hpp"

TEST(FloydWarshallTest, EmptyGraph) {
  GraphAdjacencyList<int, int> graph;
  FloydWarshall<int, int> floyd_warshall;
  floyd_warshall.set_graph(graph);
  floyd_warshall.compute();
  //EXPECT_THROW(floyd_warshall.shortest_path(0, 0).first.empty(), std::invalid_argument);
  EXPECT_THROW(floyd_warshall.shortest_path(0, 0), std::invalid_argument);
}

TEST(FloydWarshallTest, SingleVertexGraph) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  FloydWarshall<int, int> floyd_warshall;
  floyd_warshall.set_graph(graph);
  floyd_warshall.compute();
  EXPECT_TRUE(same(floyd_warshall.shortest_path(0, 0).first, std::vector<int>({0,0})));
}

TEST(FloydWarshallTest, TwoVertexGraphNoEdge) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  FloydWarshall<int, int> floyd_warshall;
  floyd_warshall.set_graph(graph);
  floyd_warshall.compute();
  auto p = floyd_warshall.shortest_path(0, 1).first;
  EXPECT_TRUE(p.empty());
}

TEST(FloydWarshallTest, TwoVertexGraphWithEdge) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  graph.add_edge(0, 1, 1);
  FloydWarshall<int, int> floyd_warshall;
  floyd_warshall.set_graph(graph);
  floyd_warshall.compute();
  EXPECT_TRUE(same(floyd_warshall.shortest_path(0, 1).first, std::vector<int>{0, 1}));
  EXPECT_EQ(floyd_warshall.shortest_path(0, 1).second, 1);
}

TEST(FloydWarshallTest, ThreeVertexGraphNoEdges) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  graph.add_vertex(2);
  FloydWarshall<int, int> floyd_warshall;
  floyd_warshall.set_graph(graph);
  floyd_warshall.compute();
  EXPECT_TRUE(floyd_warshall.shortest_path(0, 1).first.empty());
  EXPECT_TRUE(floyd_warshall.shortest_path(0, 2).first.empty());
  EXPECT_TRUE(floyd_warshall.shortest_path(1, 2).first.empty());
}

TEST(FloydWarshallTest, ThreeVertexGraphWithEdges) {
  GraphAdjacencyList<int, int> graph;
  graph.add_vertex(0);
  graph.add_vertex(1);
  graph.add_vertex(2);
  graph.add_edge(0, 1, 1);
  graph.add_edge(1, 2, 1);
  FloydWarshall<int, int> floyd_warshall;
  floyd_warshall.set_graph(graph);
  floyd_warshall.compute();
  EXPECT_TRUE(same(floyd_warshall.shortest_path(0, 1).first, std::vector<int>{0, 1}));
  EXPECT_EQ(floyd_warshall.shortest_path(0, 1).second, 1);
  EXPECT_TRUE(same(floyd_warshall.shortest_path(0, 2).first, std::vector<int>{0, 1, 2}));
  EXPECT_EQ(floyd_warshall.shortest_path(0, 2).second, 2);
  EXPECT_TRUE(same(floyd_warshall.shortest_path(1, 2).first, std::vector<int>{1, 2}));
  EXPECT_EQ(floyd_warshall.shortest_path(1, 2).second, 1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

