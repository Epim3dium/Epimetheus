#include "utils/set.hpp"
#include <iostream>
#include <map>
#include <unordered_map>
#include <set>
#include <utility>

namespace epi {
template <typename KeyType, typename LeftValue, typename RightValue>
std::set<KeyType> IntersectMaps(const std::unordered_map<KeyType, LeftValue>& left, 
              const std::unordered_map<KeyType, RightValue>& right) {
    std::set<KeyType> result;
    auto& smaller = left.size() > right.size() ? right : left;
    auto& bigger = left.size() > right.size() ? left : right;
    for(auto& itr : smaller) {
        if(bigger.contains(itr.first))
            result.insert(itr.first);
    }
    return result;
}

}
