#include <gtest/gtest.h>
#include <limits>

#include "GraphAdjacencyList.hpp"
#include "EdmondsKarp.hpp"

// Test Fixture for the Edmonds-Karp Tests
class EdmondsKarpTest : public ::testing::Test {
protected:
    GraphAdjacencyList<int, int> graph;

    void SetUp() override {
        // Set up a simple graph for testing
        graph.add_vertex(1);
        graph.add_vertex(2);
        graph.add_vertex(3);
        graph.add_vertex(4);
    }
};

// Test for basic functionality
TEST_F(EdmondsKarpTest, BasicFlow) {
    graph.add_edge(1, 2, 10);
    graph.add_edge(2, 3, 5);
    graph.add_edge(1, 3, 15);
    graph.add_edge(3, 4, 10);
    
    auto result = edmonds_karp(graph, 1, 4);
    EXPECT_EQ(result.second, 10);
}

// Test for no path from source to sink
TEST_F(EdmondsKarpTest, NoPath) {
    graph.add_edge(1, 2, 10);
    graph.add_edge(3, 4, 5);
    
    auto result = edmonds_karp(graph, 1, 4);
    EXPECT_EQ(result.second, 0); // Maximum flow should be 0
}

// Test for zero capacity edges
TEST_F(EdmondsKarpTest, ZeroCapacityEdge) {
    graph.add_edge(1, 2, 0);
    graph.add_edge(2, 3, 5);
    graph.add_edge(1, 3, 15);
    
    auto result = edmonds_karp(graph, 1, 3);
    EXPECT_EQ(result.second, 15); // Maximum flow should still be 15
}

// Test for self-loops (should not affect flow)
TEST_F(EdmondsKarpTest, SelfLoop) {
    graph.add_edge(1, 2, 10);
    graph.add_edge(1, 1, 5); // Self-loop
    graph.add_edge(2, 3, 5);
    
    auto result = edmonds_karp(graph, 1, 3);
    EXPECT_EQ(result.second, 5); // Maximum flow should be 5
}

// Test for multiple edges between the same vertices
TEST_F(EdmondsKarpTest, MultipleEdges) {
    graph.add_edge(1, 2, 10);
    graph.add_edge(1, 2, 5); // Another edge with capacity 5
    graph.add_edge(2, 3, 15);
    
    auto result = edmonds_karp(graph, 1, 3);
    EXPECT_EQ(result.second, 15); // Maximum flow should be 15
}

// Test for disconnected graph
TEST_F(EdmondsKarpTest, DisconnectedGraph) {
    graph.add_edge(1, 2, 10);
    graph.add_edge(3, 4, 5);
    
    auto result = edmonds_karp(graph, 1, 3);
    EXPECT_EQ(result.second, 0); // Maximum flow should be 0
}

// Test for edge cases with large capacities
TEST_F(EdmondsKarpTest, LargeCapacities) {
    graph.add_edge(1, 2, std::numeric_limits<int>::max());
    graph.add_edge(2, 3, std::numeric_limits<int>::max());
    graph.add_edge(1, 3, std::numeric_limits<int>::max());
    graph.add_edge(3, 4, std::numeric_limits<int>::max());
    
    auto result = edmonds_karp(graph, 1, 4);
    EXPECT_EQ(result.second, std::numeric_limits<int>::max()); // Maximum flow should be INT_MAX
}

// Test for empty graph
TEST_F(EdmondsKarpTest, EmptyGraph) {
    auto result = edmonds_karp(graph, 1, 4);
    EXPECT_EQ(result.second, 0); // Maximum flow should be 0
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
