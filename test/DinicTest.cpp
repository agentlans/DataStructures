#include <gtest/gtest.h>

#include "GraphAdjacencyList.hpp"
#include "Dinic.hpp"

// Test Fixture for Dinic's Algorithm
class DinicTest : public ::testing::Test {
protected:
    GraphAdjacencyList<int, int> capacity;

    void SetUp() override {
        // Setup can be done here if needed
    }
};

// Test for basic functionality
TEST_F(DinicTest, BasicFunctionality) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);
    capacity.add_vertex(3);
    
    capacity.add_edge(1, 2, 10);
    capacity.add_edge(2, 3, 5);
    capacity.add_edge(1, 3, 15);

    auto [flow_network, max_flow] = dinic(capacity, 1, 3);
    
    EXPECT_EQ(max_flow, 20);
}

// Test for non-existent source vertex
TEST_F(DinicTest, NonExistentSource) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);
    capacity.add_edge(1, 2, 10);

    EXPECT_THROW(dinic(capacity, 3, 2), std::invalid_argument);
}

// Test for non-existent sink vertex
TEST_F(DinicTest, NonExistentSink) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);
    capacity.add_edge(1, 2, 10);

    EXPECT_THROW(dinic(capacity, 1, 3), std::invalid_argument);
}

// Test for empty graph
TEST_F(DinicTest, EmptyGraph) {
    EXPECT_THROW(dinic(capacity, 1, 2), std::invalid_argument);
}

// Test for graph with no edges
TEST_F(DinicTest, NoEdges) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);

    auto [flow_network, max_flow] = dinic(capacity, 1, 2);
    
    EXPECT_EQ(max_flow, 0);
}

// Test for a graph with multiple paths
TEST_F(DinicTest, MultiplePaths) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);
    capacity.add_vertex(3);
    capacity.add_vertex(4);
    
    capacity.add_edge(1, 2, 10);
    capacity.add_edge(1, 3, 10);
    capacity.add_edge(2, 4, 5);
    capacity.add_edge(3, 4, 15);

    auto [flow_network, max_flow] = dinic(capacity, 1, 4);
    
    EXPECT_EQ(max_flow, 15); // 10 from 1->3 and 5 from 1->2->4
}

// Test for edge case with large capacities
TEST_F(DinicTest, LargeCapacities) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);
    capacity.add_vertex(3);
    
    capacity.add_edge(1, 2, std::numeric_limits<int>::max());
    capacity.add_edge(2, 3, std::numeric_limits<int>::max());

    auto [flow_network, max_flow] = dinic(capacity, 1, 3);
    
    EXPECT_EQ(max_flow, std::numeric_limits<int>::max());
}

// Test for corner case with zero capacity
TEST_F(DinicTest, ZeroCapacityEdge) {
    capacity.add_vertex(1);
    capacity.add_vertex(2);
    capacity.add_edge(1, 2, 0); // Edge with zero capacity

    auto [flow_network, max_flow] = dinic(capacity, 1, 2);
    
    EXPECT_EQ(max_flow, 0);
}

// Main function to run all tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
