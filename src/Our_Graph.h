#ifndef __OUR_GRAPH__
#define __OUR_GRAPH__
#include <unordered_map>
#include <unordered_set>
#include "Node.hpp"

using Graph = std::unordered_map<Node, std::unordered_set<Node>>;
#endif