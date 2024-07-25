#include "UnionFind.hpp"
#include <gtest/gtest.h>

// Test fixture for UnionFind with int type
class UnionFindIntTest : public ::testing::Test {
protected:
  UnionFind<int> uf;

  void SetUp() override {
    // Optional: You can initialize the UnionFind instance here if needed
  }
};

// Test inserting elements
TEST_F(UnionFindIntTest, InsertElements) {
  uf.insert(1);
  uf.insert(2);
  uf.insert(3);

  EXPECT_EQ(uf.get_count(), 3); // Should have 3 disjoint sets
}

// Test finding elements
TEST_F(UnionFindIntTest, FindElements) {
  uf.insert(1);
  uf.insert(2);
  uf.insert(3);

  EXPECT_EQ(uf.find(1), 1); // 1 should be its own parent
  EXPECT_EQ(uf.find(2), 2); // 2 should be its own parent
  EXPECT_EQ(uf.find(3), 3); // 3 should be its own parent
}

// Test uniting elements
TEST_F(UnionFindIntTest, UniteElements) {
  uf.insert(1);
  uf.insert(2);
  uf.unite(1, 2);

  EXPECT_TRUE(uf.connected(1, 2)); // 1 and 2 should be connected
  EXPECT_EQ(uf.get_count(), 1);    // Should have 1 disjoint set now
}

// Test connected elements
TEST_F(UnionFindIntTest, ConnectedElements) {
  uf.insert(1);
  uf.insert(2);
  uf.insert(3);
  uf.unite(1, 2);

  EXPECT_TRUE(uf.connected(1, 2));  // 1 and 2 should be connected
  EXPECT_FALSE(uf.connected(1, 3)); // 1 and 3 should not be connected
}

// Test handling of non-existent elements
TEST_F(UnionFindIntTest, NonExistentElements) {
  EXPECT_THROW(uf.find(1), std::invalid_argument); // Should throw an exception
  EXPECT_THROW(uf.connected(1, 2),
               std::invalid_argument); // Should throw an exception
}

// Test count after multiple unions
TEST_F(UnionFindIntTest, MultipleUnions) {
  uf.insert(1);
  uf.insert(2);
  uf.insert(3);
  uf.insert(4);

  uf.unite(1, 2);
  uf.unite(3, 4);
  EXPECT_EQ(uf.get_count(), 2); // Should have 2 disjoint sets

  uf.unite(2, 3);
  EXPECT_EQ(uf.get_count(), 1); // Should have 1 disjoint set now
}

// Main function to run the tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
