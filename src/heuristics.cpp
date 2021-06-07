#include <iostream>
#include <algorithm>
#include "heuristics.hpp"
#include "rooted_orienteering.h"


std::vector<Node> my_argsort(const Rewards &rewards)
{
    typedef std::pair<reward_t, Node> tmp_pair;
    std::vector<tmp_pair> pairs;
    for (Node cnt = 0; cnt < rewards.size(); cnt++)
    {
        pairs.push_back(tmp_pair(rewards[cnt], cnt));
    }
    std::sort(pairs.begin(), pairs.end(), [&](const tmp_pair &a, const tmp_pair &b) { return a.first > b.first; });

    std::vector<Node> args;
    for (auto &kv : pairs)
    {
        args.push_back(kv.second);
    }
    return args;
}

Path post_process(Path &path, const Rewards &rewards, const Matrix &distances, const Node &root_node, const distance_t distance_limit_D)
{
    std::vector<Node> args = my_argsort(rewards);
    std::unordered_set<Node> path_nodes;
    for (Node j : path)
    {
        path_nodes.insert(j);
    }
    
    distance_t path_distance = get_path_distance(path, distances, root_node);

    Path new_path = path;
    for (Node i : args)
    {
        if (path_nodes.find(i) != path_nodes.end())
            continue;
        distance_t new_distance;
        Path::iterator current_node_iterator = new_path.begin();
        while (current_node_iterator != new_path.end())
        {
            Node current_node = *current_node_iterator;
            new_distance = path_distance + distances[current_node][i];
            Path::iterator next_node_iterator = std::next(current_node_iterator);
            if (next_node_iterator != new_path.end())
            {
                Node next_node = *next_node_iterator;
                new_distance += distances[i][next_node] - distances[current_node][next_node];
            }
            if (new_distance + DISTANCE_EPSILON <= distance_limit_D)
            {
                new_path.insert(next_node_iterator, i);
                path_distance = new_distance;
                break;
            }
            ++current_node_iterator;
        }
    }
    return new_path;
}

std::pair<bool, Path> path_generation_heuristic_1(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    Path p;
    p.push_back(root_node);
    Path new_path = post_process(p, rewards, distances, root_node, distance_limit_D);
    if (new_path.size() == 1 || get_path_reward(new_path, rewards) - REWARD_EPSILON <= 1)
    {
        return std::pair<bool, Path>(false, new_path);
    }
    else
    {
        std::cout << "Path generated by first heuristic" << std::endl;
        return std::pair<bool, Path>(true, new_path);
    }
}

std::pair<bool, Path> path_generation_heuristic_2(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    Path best_path;
    reward_t highest_reward = 0;
    for (Node i : vertices)
    {
        for (Node j : vertices)
        {
            if (i == j || i == root_node || j == root_node)
                continue;
            if (rewards[i] + rewards[j] - REWARD_EPSILON <= 1)
                continue;
            if (distances[root_node][i] + distances[i][j] > distance_limit_D + DISTANCE_EPSILON)
                continue;
            Path initial_path;
            initial_path.push_back(root_node);
            initial_path.push_back(i);
            initial_path.push_back(j);
            Path improved_path = post_process(initial_path, rewards, distances, root_node, distance_limit_D);
            reward_t reward = get_path_reward(improved_path, rewards);
            if (reward > highest_reward)
            {
                best_path = improved_path;
                highest_reward = reward;
            }
        }
    }
    if (highest_reward - REWARD_EPSILON <= 1)
    {
        return std::pair<bool, Path>(false, best_path);
    }
    else
    {
        std ::cout << "Path generated by second heuristic" << std::endl;
        return std::pair<bool, Path>(true, best_path);
    }
}

std::pair<bool, Path> path_generation_heuristic_3(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node)
{
    Path p;
    std::unordered_set<Node> path_nodes;
    p.push_back(root_node);
    path_nodes.insert(root_node);
    Node previous_node = root_node;
    distance_t path_distance = 0, new_distance;
    bool can_add_nodes = true;
    do
    {
        can_add_nodes = false;
        distance_t max_reward_to_distance_ratio = 0;
        Node argmax = root_node;
        for (Node i : vertices)
        {
            if (path_nodes.find(i) != path_nodes.end())
                continue;
            new_distance = path_distance + distances[previous_node][i];
            if (new_distance + DISTANCE_EPSILON > distance_limit_D)
                continue;
            can_add_nodes = true;
            // Pay attention to the datatypes when doing the division
            if (distances[previous_node][i] == 0){
                argmax = i;
                break;
            }
            else{
                if (max_reward_to_distance_ratio <= rewards[i] / distances[previous_node][i])
                {
                    max_reward_to_distance_ratio = rewards[i] / distances[previous_node][i];
                    argmax = i;
                }
            }
        }
        if (can_add_nodes)
        {
            p.push_back(argmax);
            path_nodes.insert(argmax);
            path_distance += distances[previous_node][argmax];
            previous_node = argmax;
        }
    } while (can_add_nodes);

    if (p.size() == 1 || get_path_reward(p, rewards) - REWARD_EPSILON <= 1)
    {
        return std::pair<bool, Path>(false, p);
    }
    else
    {
        std::cout << "Path generated by third heuristic" << std::endl;
        return std::pair<bool, Path>(true, p);
    }
}