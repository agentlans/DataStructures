#include <gtest/gtest.h>

#include "GraphAdjacencyList.hpp"
#include "Kruskal.hpp"

// Test Fixture for Kruskal's Algorithm
class KruskalTest : public ::testing::Test {
protected:
    // Example graph for testing
    GraphAdjacencyList<int, int> graph;

    void SetUp() override {
        // Setup a simple graph with vertices and edges
        graph.add_vertex(1);
        graph.add_vertex(2);
        graph.add_vertex(3);
        graph.add_vertex(4);
        
        graph.add_biedge(1, 2, 1);
        graph.add_biedge(1, 3, 3);
        graph.add_biedge(2, 3, 2);
        graph.add_biedge(2, 4, 4);
        graph.add_biedge(3, 4, 5);
    }
};

// Test case for a basic graph
TEST_F(KruskalTest, BasicGraph) {
    GraphAdjacencyList<int, int> mst = kruskal(&graph);
    graph.print_graph();
    std::cout << "MST" << std::endl;
    mst.print_graph();

    // Check that the MST has the correct number of edges
    EXPECT_EQ(mst.get_vertices().size(), 4); // Should have 4 vertices
    EXPECT_EQ(mst.get_neighbours(1).size(), 1); // Vertex 1 should connect to 2
    EXPECT_EQ(mst.get_neighbours(2).size(), 3); // Vertex 2 should connect to 1, 3, and 4
    EXPECT_EQ(mst.get_neighbours(3).size(), 1); // Vertex 3 should connect to 2
    EXPECT_EQ(mst.get_neighbours(4).size(), 1); // Vertex 4 should connect to 2
}

// Test case for negative weights
TEST(KruskalNegativeWeightTest, ThrowsException) {
    GraphAdjacencyList<int, int> negative_graph;
    negative_graph.add_vertex(1);
    negative_graph.add_vertex(2);
    negative_graph.add_edge(1, 2, -1); // Negative weight edge

    EXPECT_THROW(kruskal(&negative_graph), std::invalid_argument);
}

// Test case for a graph with no edges
TEST(KruskalNoEdgesTest, EmptyMST) {
    GraphAdjacencyList<int, int> empty_graph;
    empty_graph.add_vertex(1);
    empty_graph.add_vertex(2);

    GraphAdjacencyList<int, int> mst = kruskal(&empty_graph);

    // Check that the MST has no edges
    EXPECT_EQ(mst.get_vertices().size(), 0); // Should have no vertices
    EXPECT_EQ(mst.get_neighbours(1).size(), 0); // No edges
    EXPECT_EQ(mst.get_neighbours(2).size(), 0); // No edges
}

// Test case for a fully connected graph
TEST(KruskalFullyConnectedTest, FullGraph) {
    GraphAdjacencyList<int, int> full_graph;
    full_graph.add_vertex(1);
    full_graph.add_vertex(2);
    full_graph.add_vertex(3);
    full_graph.add_vertex(4);

    full_graph.add_biedge(1, 2, 1);
    full_graph.add_biedge(1, 3, 3);
    full_graph.add_biedge(1, 4, 4);
    full_graph.add_biedge(2, 3, 2);
    full_graph.add_biedge(2, 4, 5);
    full_graph.add_biedge(3, 4, 6);

    GraphAdjacencyList<int, int> mst = kruskal(&full_graph);
    full_graph.print_graph();
    std::cout << "MST" << std::endl;
    mst.print_graph();

    // Check that the MST has the correct number of edges
    EXPECT_EQ(mst.get_vertices().size(), 4); // Should have 4 vertices
    EXPECT_EQ(mst.get_neighbours(1).size(), 2); // Should connect to 2 and 4
    EXPECT_EQ(mst.get_neighbours(2).size(), 2); // Should connect to 1 and 3
    EXPECT_EQ(mst.get_neighbours(3).size(), 1); // Should connect to 2
    EXPECT_EQ(mst.get_neighbours(4).size(), 1); // Should connect to 1
}

// Main function for running the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

