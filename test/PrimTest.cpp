#include <gtest/gtest.h>

#include "GraphAdjacencyList.hpp"
#include "Prim.hpp"

class PrimTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code to initialize the graph for testing
        graph = new GraphAdjacencyList<int, int>();
        // Add vertices and edges to the graph
        graph->add_vertex(1);
        graph->add_vertex(2);
        graph->add_vertex(3);
        graph->add_biedge(1, 2, 1);
        graph->add_biedge(2, 3, 2);
        graph->add_biedge(1, 3, 3);
    }

    void TearDown() override {
        delete graph; // Clean up the graph after each test
    }

    GraphAdjacencyList<int, int>* graph;
};

TEST_F(PrimTest, TestMinimumSpanningTree) {
    GraphAdjacencyList<int, int> mst = prim(graph);
    graph->print_graph();
	std::cout << "MST" << std::endl;
    mst.print_graph();

    // Check that the MST contains the correct edges
    EXPECT_TRUE(mst.has_edge(1, 2));
    EXPECT_TRUE(mst.has_edge(2, 3));
    EXPECT_FALSE(mst.has_edge(1, 3)); // The edge (1, 3) should not be in the MST

    // Check the weights of the edges in the MST
    EXPECT_EQ(mst.get_weight(1, 2), 1);
    EXPECT_EQ(mst.get_weight(2, 3), 2);
}

TEST_F(PrimTest, TestEmptyGraph) {
    GraphAdjacencyList<int, int> empty_graph;
    GraphAdjacencyList<int, int> mst = prim(&empty_graph);

    // The MST of an empty graph should also be empty
    EXPECT_TRUE(mst.get_vertices().empty());
}

TEST_F(PrimTest, TestSingleVertexGraph) {
    GraphAdjacencyList<int, int> single_vertex_graph;
    single_vertex_graph.add_vertex(1);
    GraphAdjacencyList<int, int> mst = prim(&single_vertex_graph);

    // The MST of a single vertex graph should be itself
    EXPECT_TRUE(mst.get_vertices().size() == 1);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

