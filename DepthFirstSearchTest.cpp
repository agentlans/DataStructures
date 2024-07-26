#include <gtest/gtest.h>
#include "GraphAdjacencyList.hpp"
#include "DepthFirstSearch.hpp"

/**
 * @brief Test fixture for DepthFirstSearch tests.
 */
class DepthFirstSearchTest : public ::testing::Test {
protected:
    GraphAdjacencyList<int, int> directed_graph;
    GraphAdjacencyList<int, int> undirected_graph;
    DepthFirstSearch<int, int> dfs;

    void SetUp() override {
        // Set up a sample directed graph for testing
        directed_graph.add_vertex(1);
        directed_graph.add_vertex(2);
        directed_graph.add_vertex(3);
        directed_graph.add_vertex(4);
        directed_graph.add_edge(1, 2, 1);
        directed_graph.add_edge(1, 3, 1);
        directed_graph.add_edge(2, 4, 1);
        directed_graph.add_edge(3, 4, 1);
        dfs.set_graph(directed_graph);

        // Set up a sample undirected graph for bipartiteness testing
        undirected_graph.add_vertex(1);
        undirected_graph.add_vertex(2);
        undirected_graph.add_vertex(3);
        undirected_graph.add_vertex(4);
        undirected_graph.add_biedge(1, 2, 1);
        undirected_graph.add_biedge(1, 3, 1);
        undirected_graph.add_biedge(2, 4, 1);
        undirected_graph.add_biedge(3, 4, 1);
    }
};

/**
 * @brief Test the DFS traversal.
 */
TEST_F(DepthFirstSearchTest, TestDFS) {
    std::unordered_set<int> visited = dfs.dfs(1);
    EXPECT_TRUE(visited.find(1) != visited.end());
    EXPECT_TRUE(visited.find(2) != visited.end());
    EXPECT_TRUE(visited.find(3) != visited.end());
    EXPECT_TRUE(visited.find(4) != visited.end());
}

/**
 * @brief Test finding a path between two vertices.
 */
TEST_F(DepthFirstSearchTest, TestPath) {
    std::vector<int> path = dfs.path(1, 4);
    EXPECT_EQ(path.back(), 4); // Ensure the last vertex is the target
    EXPECT_EQ(path.front(), 1); // Ensure the first vertex is the start

    // Test path that does not exist
    std::vector<int> no_path = dfs.path(3, 1);
    EXPECT_TRUE(no_path.empty()); // Ensure no path found
}

/**
 * @brief Test cycle detection in the directed graph.
 */
TEST_F(DepthFirstSearchTest, TestCycleDetection) {
    directed_graph.add_edge(4, 2, 1); // Create a cycle 2 -> 4 -> 2
    std::vector<int> cycle = dfs.find_cycle();
    EXPECT_FALSE(cycle.empty()); // Ensure a cycle is found
}

/**
 * @brief Test topological sorting.
 */
TEST_F(DepthFirstSearchTest, TestTopologicalSort) {
    std::vector<int> sorted = dfs.topological_sort();
    EXPECT_EQ(sorted.size(), 4); // Ensure all vertices are included
    EXPECT_NE(std::find(sorted.begin(), sorted.end(), 4), sorted.end()); // Ensure vertex 4 is present

    // Test topological sort on a graph with a cycle
    directed_graph.add_edge(4, 1, 1); // Create a cycle
    directed_graph.print_graph();
    EXPECT_THROW(dfs.topological_sort(), std::runtime_error); // Should throw an error
}

/**
 * @brief Test bipartiteness of the undirected graph.
 */
TEST_F(DepthFirstSearchTest, TestIsBipartite) {
    dfs.set_graph(undirected_graph);
    EXPECT_TRUE(dfs.is_bipartite()); // The graph should be bipartite

    // Create a non-bipartite graph (odd cycle)
    undirected_graph.add_biedge(2, 3, 1); // Create an odd cycle 1-2-3-1
    EXPECT_FALSE(dfs.is_bipartite()); // The graph should not be bipartite
}

/**
 * @brief Test with an empty graph.
 */
TEST_F(DepthFirstSearchTest, TestEmptyGraph) {
    GraphAdjacencyList<int, int> empty_graph;
    DepthFirstSearch<int, int> empty_dfs;
    empty_dfs.set_graph(empty_graph);

    EXPECT_TRUE(empty_dfs.dfs(1).empty());
    EXPECT_TRUE(empty_dfs.path(1, 2).empty()); // No path should exist
}

/**
 * @brief Test with a single vertex.
 */
TEST_F(DepthFirstSearchTest, TestSingleVertex) {
    GraphAdjacencyList<int, int> single_vertex_graph;
    single_vertex_graph.add_vertex(1);
    DepthFirstSearch<int, int> single_vertex_dfs;
    single_vertex_dfs.set_graph(single_vertex_graph);

    EXPECT_EQ(single_vertex_dfs.dfs(1).size(), 1); // Should only visit the single vertex
    EXPECT_TRUE(single_vertex_dfs.path(1, 2).empty()); // No path should exist
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
