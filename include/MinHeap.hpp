#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include "BijectiveMap.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>

/**
 * @class MinHeap
 * @brief A template-based minimum heap implementation.
 *
 * This class provides a priority queue structure where the minimum element
 * can be efficiently accessed and removed. It supports insertion, updating,
 * and retrieval of elements based on their keys.
 *
 * @tparam KeyType The type of the keys used to identify elements in the heap.
 * @tparam ValueType The type of the values stored in the heap.
 */
template <class KeyType, class ValueType> class MinHeap {
public:
  /**
   * @brief Retrieves the value associated with a given key.
   *
   * This method searches for the specified key in the heap and returns
   * its corresponding value.
   *
   * @param key The key whose associated value is to be retrieved.
   * @return The value associated with the specified key.
   * @throw std::out_of_range if the key is not found in the heap.
   */
  ValueType get(const KeyType &key) const;

  /**
   * @brief Sets the value for a specified key.
   *
   * This method inserts a new key-value pair into the heap or updates
   * the value if the key already exists.
   *
   * @param key The key to be set or updated.
   * @param value The value to be associated with the key.
   */
  void set(const KeyType &key, const ValueType &value);

  /**
   * @brief Retrieves the minimum key-value pair in the heap.
   *
   * This method returns the key-value pair with the minimum value
   * without removing it from the heap.
   *
   * @return A pair containing the minimum key and its associated value.
   * @throw std::underflow_error if the heap is empty.
   */
  std::pair<KeyType, ValueType> get_min() const;

  /**
   * @brief Extracts and removes the minimum value from the heap.
   *
   * This method removes the minimum key-value pair from the heap
   * and returns its value.
   *
   * @return The value of the extracted minimum key-value pair.
   * @throw std::underflow_error if the heap is empty.
   */
  std::pair<KeyType, ValueType> extract_min();

  /**
   * @brief Checks if the heap is empty.
   *
   * This method returns true if the heap contains no elements,
   * otherwise false.
   *
   * @return True if the heap is empty; otherwise false.
   */
  bool is_empty() const;

  /**
   * @brief Returns the number of elements in the heap.
   *
   * This method provides the current size of the heap.
   *
   * @return The number of elements in the heap.
   */
  int size() const;

  /**
   * @brief Prints the contents of the heap.
   *
   * This method outputs the elements of the heap to the console
   * in a human-readable format.
   */
  void print_heap() const;

private:
  BijectiveMap<KeyType, int>
      handles; ///< Maps keys to their indices in the heap.
  std::vector<ValueType>
      heap; ///< The underlying vector storing the heap elements.

  /**
   * @brief Restores the heap property by moving an element up the heap.
   *
   * This method is called after an insertion or update to ensure that
   * the heap maintains its minimum property.
   *
   * @param index The index of the element to be heapified up.
   */
  void heapify_up(int index);

  /**
   * @brief Restores the heap property by moving an element down the heap.
   *
   * This method is called after an extraction to ensure that the heap
   * maintains its minimum property.
   *
   * @param index The index of the element to be heapified down.
   */
  void heapify_down(int index);

  /**
   * @brief Inserts a new key-value pair into the heap.
   *
   * This method adds a new element to the heap and ensures that the
   * heap property is maintained.
   *
   * @param key The key of the new element.
   * @param value The value of the new element.
   */
  void insert(const KeyType &key, ValueType value);

  /**
   * @brief Updates the value of an existing key in the heap.
   *
   * This method modifies the value associated with a given key and
   * ensures that the heap property is maintained.
   *
   * @param handle The key whose value is to be updated.
   * @param new_value The new value to be associated with the key.
   * @throw std::out_of_range if the key is not found in the heap.
   */
  void update(const KeyType &handle, ValueType new_value);
};

template <class KeyType, class ValueType>
ValueType MinHeap<KeyType, ValueType>::get(const KeyType &key) const {
  try {
    int index = handles.get_by_key(key);
    return heap[index];
  } catch (const std::runtime_error &e) {
    throw std::out_of_range("Key not found.");
    return ValueType();
  }
}

template <class KeyType, class ValueType>
void MinHeap<KeyType, ValueType>::set(const KeyType &key,
                                      const ValueType &value) {
  if (handles.contains_key(key)) {
    update(key, value);
  } else {
    insert(key, value);
  }
}

template <class KeyType, class ValueType>
void MinHeap<KeyType, ValueType>::heapify_up(int index) {
  while (index > 0) {
    int parent_index = (index - 1) / 2;
    if (heap[index] < heap[parent_index]) {
      std::swap(heap[index], heap[parent_index]);
      handles.swap_by_value(index, parent_index);
      index = parent_index;
    } else {
      break;
    }
  }
}

template <class KeyType, class ValueType>
void MinHeap<KeyType, ValueType>::heapify_down(int index) {
  int size = heap.size();
  while (index < size) {
    int left_child = 2 * index + 1;
    int right_child = 2 * index + 2;
    int smallest = index;

    if (left_child < size && heap[left_child] < heap[smallest]) {
      smallest = left_child;
    }
    if (right_child < size && heap[right_child] < heap[smallest]) {
      smallest = right_child;
    }
    if (smallest != index) {
      std::swap(heap[index], heap[smallest]);
      handles.swap_by_value(index, smallest);
      index = smallest;
    } else {
      break;
    }
  }
}

template <class KeyType, class ValueType>
void MinHeap<KeyType, ValueType>::insert(const KeyType &key, ValueType value) {
  heap.push_back(value);           // Add the new value to the end of the heap
  int new_index = heap.size() - 1; // Get the index of the newly added value
  handles.put(key, new_index); // Store the handle and its corresponding index
                               // in the BijectiveMap
  heapify_up(new_index); // Restore the heap property by moving the new value up
}

template <class KeyType, class ValueType>
std::pair<KeyType, ValueType> MinHeap<KeyType, ValueType>::extract_min() {
  if (heap.empty()) {
    throw std::underflow_error("Heap is empty");
  }

  ValueType min_value = heap[0];
  KeyType min_handle = handles.get_by_value(0);

  heap[0] = heap.back();
  handles.swap_by_value(0, size() - 1);
  heap.pop_back();
  handles.remove_by_key(min_handle);

  if (!heap.empty()) {
    heapify_down(0);
  }

  return {min_handle, min_value};
}

template <class KeyType, class ValueType>
std::pair<KeyType, ValueType> MinHeap<KeyType, ValueType>::get_min() const {
  if (heap.empty()) {
    throw std::underflow_error("Heap is empty");
  }
  return {handles.get_by_value(0), heap[0]};
}

template <class KeyType, class ValueType>
void MinHeap<KeyType, ValueType>::update(const KeyType &handle,
                                         ValueType new_value) {
  int index = handles.get_by_key(handle);
  if (index < 0 || index >= heap.size()) {
    throw std::runtime_error("Invalid handle");
  }

  ValueType old_value = heap[index];
  heap[index] = new_value;

  if (new_value < old_value) {
    heapify_up(index);
  } else {
    heapify_down(index);
  }
}

template <class KeyType, class ValueType>
bool MinHeap<KeyType, ValueType>::is_empty() const {
  return heap.empty();
}

template <class KeyType, class ValueType>
int MinHeap<KeyType, ValueType>::size() const {
  return heap.size();
}

template <class KeyType, class ValueType>
void MinHeap<KeyType, ValueType>::print_heap() const {
  for (const ValueType &value : heap) {
    std::cout << value << " ";
  }
  std::cout << std::endl;

  std::cout << "Handles:" << std::endl;
  handles.display();
}

#endif // MIN_HEAP_H
