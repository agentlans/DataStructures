#include "MinHeap.hpp"
#include <gtest/gtest.h>

// g++ Test.cpp `pkg-config --cflags --libs gtest`

class MinHeapTest : public ::testing::Test {
protected:
  MinHeap<int, int> minHeap;

  void SetUp() override {
    // Initialize the heap with some values
    minHeap.set(1, 10);
    minHeap.set(2, 20);
    minHeap.set(3, 15);
  }
};

TEST_F(MinHeapTest, TestInsertAndGetMin) {
  minHeap.set(4, 5);
  EXPECT_EQ(minHeap.get_min().second, 5); // The minimum value should be 5
}

TEST_F(MinHeapTest, TestExtractMin) {
  auto minPair = minHeap.extract_min();
  EXPECT_EQ(minPair.second, 10); // Should return 10, the minimum value
  EXPECT_EQ(minHeap.get_min().second, 15); // Now the new minimum should be 15
}

TEST_F(MinHeapTest, TestIsEmpty) {
  EXPECT_FALSE(minHeap.is_empty()); // Should not be empty after setup
  minHeap.extract_min();            // Remove one element
  minHeap.extract_min();            // Remove another
  minHeap.extract_min();            // Remove the last element
  EXPECT_TRUE(minHeap.is_empty());  // Now it should be empty
}

TEST_F(MinHeapTest, TestSize) {
  EXPECT_EQ(minHeap.size(), 3); // Initial size should be 3
  minHeap.set(4, 5);
  EXPECT_EQ(minHeap.size(), 4); // Size should increase after insertion
  minHeap.extract_min();
  EXPECT_EQ(minHeap.size(), 3); // Size should decrease after extraction
}

TEST_F(MinHeapTest, TestUpdateValue) {
  minHeap.set(1, 12);                      // Update key 1's value to 12
  EXPECT_EQ(minHeap.get(1), 12);           // Should return updated value
  EXPECT_EQ(minHeap.get_min().second, 12); // Minimum should now be 12
}

TEST_F(MinHeapTest, TestGetThrowsOnEmpty) {
  MinHeap<int, int> emptyHeap;
  EXPECT_THROW(emptyHeap.get(1), std::out_of_range); // Should throw when key is not found
}

TEST_F(MinHeapTest, TestExtractMinThrowsOnEmpty) {
  MinHeap<int, int> emptyHeap;
  EXPECT_THROW(emptyHeap.extract_min(), std::underflow_error); // Should throw when extracting from empty heap
}

TEST_F(MinHeapTest, TestPrintHeap) {
  // This test is more for manual verification,
  // you could redirect output or check internal state if needed.
  minHeap.print_heap(); // Just to see if it prints without crashing
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
