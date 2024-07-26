#include "UnorderedContainers.hpp"

int main() {
    //std::unordered_map<std::string, std::string, Hash<std::string>> regularMap;
    //std::unordered_map<std::pair<int, int>, std::string, Hash<std::pair<int, int>>> pairMap;
	UnorderedMap<std::string, std::string> regularMap;
	UnorderedMap<std::pair<int,int>, std::string> pairMap;

    // Inserting into the regular map
    regularMap["key1"] = "value1";
    regularMap["key2"] = "value2";

    // Inserting into the pair map
    pairMap[std::make_pair(1, 2)] = "Value for (1, 2)";
    pairMap[std::make_pair(3, 4)] = "Value for (3, 4)";

    // Accessing elements
    std::cout << regularMap["key1"] << std::endl; // Outputs: value1
    std::cout << pairMap[std::make_pair(1, 2)] << std::endl; // Outputs: Value for (1, 2)

    return 0;
}
