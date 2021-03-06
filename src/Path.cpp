#include "Path.h"
#include <unordered_set>

penalty_t get_path_reward(const Path &p, const Rewards &rewards)
{
    penalty_t path_reward = 0;
    std::unordered_set <Node> node_set;
    for (Node n : p)
    {
        node_set.insert(n);
        // path_reward += rewards[_];
    }
    for (Node n: node_set){
        path_reward += rewards[n];
    }
    return path_reward;
}

distance_t get_path_distance(const Path &p, const Matrix &distances)
{
    distance_t path_distance = 0;
    for (auto node_it = p.begin(); node_it != p.end(); node_it++)
    {
        if (std::next(node_it, 1) == p.end()) break;
        Node current_node = *node_it;
        Node next_node = *std::next(node_it, 1);
        path_distance += distances[current_node][next_node];
        // previous_node = n;
    }
    return path_distance;
}