#ifndef BIJECTIVE_MAP_HPP
#define BIJECTIVE_MAP_HPP

#include <iostream>
#include <stdexcept>
#include "UnorderedContainers.hpp"

template <class KeyType, class ValueType> class BijectiveMap {
public:
  // Create or Update an entry in the bijective map
  void put(const KeyType &key, const ValueType &value);

  // Read an entry by key
  ValueType get_by_key(const KeyType &key) const;

  // Read an entry by value
  KeyType get_by_value(const ValueType &value) const;

  // Delete an entry by key
  void remove_by_key(const KeyType &key);

  // Delete an entry by value
  void remove_by_value(const ValueType &value);

  // Swap values of two keys
  void swap_by_key(const KeyType &key1, const KeyType &key2);

  // Swap keys of two values
  void swap_by_value(const ValueType &value1, const ValueType &value2);

  bool contains_key(const KeyType &key) const;

  // Display the contents of the bijective map
  void display() const;

private:
  UnorderedMap<KeyType, ValueType> forward_map; // Maps keys to values
  UnorderedMap<ValueType, KeyType> reverse_map; // Maps values to keys
};

// Create or Update an entry in the bijective map
template <class KeyType, class ValueType>
bool BijectiveMap<KeyType, ValueType>::contains_key(const KeyType &key) const {
  return (forward_map.find(key) != forward_map.end());
}

// Create or Update an entry in the bijective map
template <class KeyType, class ValueType>
void BijectiveMap<KeyType, ValueType>::put(const KeyType &key,
                                           const ValueType &value) {
  // Check if the key already exists in the forward map
  if (forward_map.find(key) != forward_map.end()) {
    // If the key exists, remove the old value from reverse_map
    ValueType old_value = forward_map[key];
    reverse_map.erase(old_value);
  }

  // Check if the value already exists in the reverse map
  if (reverse_map.find(value) != reverse_map.end()) {
    // If the value exists, remove the old key from forward_map
    KeyType old_key = reverse_map[value];
    forward_map.erase(old_key);
  }

  // Insert the new key-value pair
  forward_map[key] = value;
  reverse_map[value] = key;
}

// Read an entry by key
template <class KeyType, class ValueType>
ValueType
BijectiveMap<KeyType, ValueType>::get_by_key(const KeyType &key) const {
  auto it = forward_map.find(key);
  if (it == forward_map.end()) {
    throw std::runtime_error("Key not found");
  }
  return it->second;
}

// Read an entry by value
template <class KeyType, class ValueType>
KeyType
BijectiveMap<KeyType, ValueType>::get_by_value(const ValueType &value) const {
  auto it = reverse_map.find(value);
  if (it == reverse_map.end()) {
    throw std::runtime_error("Value not found");
  }
  return it->second;
}

// Delete an entry by key
template <class KeyType, class ValueType>
void BijectiveMap<KeyType, ValueType>::remove_by_key(const KeyType &key) {
  auto it = forward_map.find(key);
  if (it == forward_map.end()) {
    throw std::runtime_error("Key not found");
  }
  ValueType value = it->second;
  forward_map.erase(it);
  reverse_map.erase(value);
}

// Delete an entry by value
template <class KeyType, class ValueType>
void BijectiveMap<KeyType, ValueType>::remove_by_value(const ValueType &value) {
  auto it = reverse_map.find(value);
  if (it == reverse_map.end()) {
    throw std::runtime_error("Value not found");
  }
  KeyType key = it->second;
  reverse_map.erase(it);
  forward_map.erase(key);
}

// Swap values of two keys
template <class KeyType, class ValueType>
void BijectiveMap<KeyType, ValueType>::swap_by_key(const KeyType &key1,
                                                   const KeyType &key2) {
  auto it1 = forward_map.find(key1);
  auto it2 = forward_map.find(key2);

  if (it1 == forward_map.end() || it2 == forward_map.end()) {
    throw std::runtime_error("One or both keys not found");
  }

  ValueType value1 = it1->second;
  ValueType value2 = it2->second;

  // Swap in forward_map
  forward_map[key1] = value2;
  forward_map[key2] = value1;

  // Update reverse_map accordingly
  reverse_map[value1] = key2;
  reverse_map[value2] = key1;
}

// Swap keys of two values
template <class KeyType, class ValueType>
void BijectiveMap<KeyType, ValueType>::swap_by_value(const ValueType &value1,
                                                     const ValueType &value2) {
  auto it1 = reverse_map.find(value1);
  auto it2 = reverse_map.find(value2);

  if (it1 == reverse_map.end() || it2 == reverse_map.end()) {
    throw std::runtime_error("One or both values not found");
  }

  KeyType key1 = it1->second;
  KeyType key2 = it2->second;

  // Swap in forward_map
  forward_map[key1] = value2;
  forward_map[key2] = value1;

  // Update reverse_map accordingly
  reverse_map[value1] = key2;
  reverse_map[value2] = key1;
}

// Display the contents of the bijective map
template <class KeyType, class ValueType>
void BijectiveMap<KeyType, ValueType>::display() const {
  std::cout << "Bijective Map Contents:\n";
  for (const auto &pair : forward_map) {
    std::cout << "Key: " << pair.first << ", Value: " << pair.second << "\n";
  }

  std::cout << "Reverse Bijective Map Contents:\n";
  for (const auto &pair : reverse_map) {
    std::cout << "Value: " << pair.first << ", Key: " << pair.second << "\n";
  }
  std::cout << std::endl;
}

#endif