#ifndef UNIONFIND_H
#define UNIONFIND_H

#include <stdexcept>
#include <unordered_map>

template <class T> class UnionFind {
public:
  // Constructor
  UnionFind();

  /**
   * Inserts a new element into the union-find structure.
   * If the element already exists, it does nothing.
   *
   * @param x The element to insert.
   */
  void insert(const T &x);

  /**
   * Finds the representative (root) of the set containing x.
   * Implements path compression for efficiency.
   *
   * @param x The element whose representative is to be found.
   * @return The representative of the set containing x.
   * @throws std::invalid_argument if x is not found.
   */
  T find(const T &x);

  /**
   * Unites the sets containing elements x and y.
   * Uses union by rank to attach the smaller tree under the larger tree.
   *
   * @param x The first element.
   * @param y The second element.
   */
  void unite(const T &x, const T &y);

  /**
   * Checks if elements x and y are in the same set.
   *
   * @param x The first element.
   * @param y The second element.
   * @return True if x and y are connected, false otherwise.
   */
  bool connected(const T &x, const T &y);

  /**
   * Returns the number of disjoint sets currently in the union-find structure.
   *
   * @return The number of disjoint sets.
   */
  int get_count() const;

private:
  std::unordered_map<T, T> parent; // Maps each element to its parent
  std::unordered_map<T, int> rank; // Maps each element to its rank
  int count;                       // Number of disjoint sets
};

// Constructor
template <class T> UnionFind<T>::UnionFind() : count(0) {}

// Inserts a new element into the union-find structure.
template <class T> void UnionFind<T>::insert(const T &x) {
  if (parent.find(x) == parent.end()) {
    parent[x] = x; // Each element is its own parent
    rank[x] = 0;   // Initial rank is 0
    count++;       // Increase the count of disjoint sets
  }
}

// Finds the representative (root) of the set containing x.
template <class T> T UnionFind<T>::find(const T &x) {
  if (parent.find(x) == parent.end()) {
    throw std::invalid_argument("Element not found");
  }
  if (parent[x] != x) {
    parent[x] = find(parent[x]); // Path compression
  }
  return parent[x];
}

// Unites the sets containing elements x and y.
template <class T> void UnionFind<T>::unite(const T &x, const T &y) {
  // insert(x); // Ensure both elements are in the structure
  // insert(y);

  T rootX = find(x);
  T rootY = find(y);

  if (rootX != rootY) {
    // Union by rank
    if (rank[rootX] < rank[rootY]) {
      parent[rootX] = rootY;
    } else if (rank[rootX] > rank[rootY]) {
      parent[rootY] = rootX;
    } else {
      parent[rootY] = rootX;
      rank[rootX]++;
    }
    count--; // Decrease the count of disjoint sets
  }
}

// Checks if elements x and y are in the same set.
template <class T> bool UnionFind<T>::connected(const T &x, const T &y) {
  if (parent.find(x) == parent.end() || parent.find(y) == parent.end()) {
    throw std::invalid_argument("One or both elements not found");
  }
  return find(x) == find(y);
}

// Returns the number of disjoint sets currently in the union-find structure.
template <class T> int UnionFind<T>::get_count() const { return count; }

#endif // UNIONFIND_H
