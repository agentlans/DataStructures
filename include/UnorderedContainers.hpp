#ifndef _UNORDEREDCONTAINERS
#define _UNORDEREDCONTAINERS

#include <unordered_map>
#include <unordered_set>
#include <utility>

template <typename T>
struct Hash {
    std::size_t operator()(const T& key) const {
        return std::hash<T>()(key); // Default hash for regular types
    }
};

template <typename T1, typename T2>
struct Hash<std::pair<T1, T2>> {
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>()(p.first);
        auto hash2 = std::hash<T2>()(p.second);
        return hash1 ^ (hash2 << 1); // Combine the two hash values
    }
};

// Type for unordered maps
template <typename KeyType, typename ValueType>
using UnorderedMap = std::unordered_map<KeyType, ValueType, Hash<KeyType>>;

// Type for unordered sets
template <typename T>
using UnorderedSet = std::unordered_set<T, Hash<T>>;

#endif