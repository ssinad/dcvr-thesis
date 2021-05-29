#include "orienteering.h"
#include "iterPCA.hpp"
#include <stack>
#include <unordered_set>
#include "Our_Graph.h"
#include <assert.h>

#ifndef NDEBUG
#include <iostream>
#endif

const double EPS = 1e-6;

distance_t edge_cost(const Arborescence &arb_T, const Matrix &costs)
{
    distance_t sum = 0;
    for (std::pair<Node, Node> kv : arb_T)
    {
        Node u = kv.second;
        Node v = kv.first;
        sum += costs[u][v];
    }
    return sum;
}

penalty_t total_reward(const Arborescence &arb_T, const Penalties &penalties)
{
    penalty_t tot = 0;
    for (std::pair<Node, Node> kv : arb_T)
    {
        tot += penalties[kv.first];
    }
    return tot;
}

void binary_search_recursive(Arborescence &a1, Arborescence &a2, const Vertices &vertices, const Matrix &costs, penalty_t &lambda_1, penalty_t &lambda_2, penalty_t &theta_1, penalty_t &theta_2, const Penalties &penalties, int num_nodes, const Node &root_node, const Node &t, distance_t distance_limit_D)
{
    if (lambda_1 + EPS >= lambda_2)
    {
        return;
    }

    Matrix c1 = Matrix(3 * num_nodes, std::vector<distance_t>(3 * num_nodes, 0));
    for (int row = 0; row < num_nodes; row++)
    {
        for (int col = 0; col < num_nodes; col++)
        {
            c1[row][col] = costs[row][col];
        }
    }
    Vertices v1 = vertices;
    penalty_t lambda = (lambda_1 + lambda_2) / 2.0;

    Penalties p1(2 * num_nodes, 0);
    for (int cnt = 0; cnt < num_nodes; cnt++)
    {
        p1[cnt] = penalties[cnt];
    }
    
    for (int cnt = 0; cnt < penalties.size(); cnt++)
    {
        p1[cnt] *= lambda;
    }
    p1[t] = 1 + costs[root_node][t];
    assert(costs[root_node][t] <= distance_limit_D);
    penalty_t theta = 0; //, theta_1, theta_2;
    Arborescence a = iterPCA_with_check(v1, c1, p1, theta, num_nodes - 1, root_node);

    distance_t tree_cost = edge_cost(a, costs);
    if (tree_cost - distance_limit_D <= EPS && tree_cost - distance_limit_D >= -EPS)
    {
        theta_1 = theta;
        theta_2 = theta;
        lambda_1 = lambda_2 = lambda;
        a1 = a;
        a2 = a;
        return;
    }
    else
    {
        if (tree_cost >= distance_limit_D + EPS)
        {
            a2 = a;
            theta_2 = theta;
            lambda_2 = lambda;
            // TODO Should this be the original penalties, costs, vertices?
            binary_search_recursive(a1, a2, vertices, costs, lambda_1, lambda_2, theta_1, theta_2, penalties, num_nodes, root_node, t, distance_limit_D);
            return;
        }
        else
        {
            a1 = a;
            theta_1 = theta;
            lambda_1 = lambda;
            binary_search_recursive(a1, a2, vertices, costs, lambda_1, lambda_2, theta_1, theta_2, penalties, num_nodes, root_node, t, distance_limit_D);
            return;
        }
    }
}

void binary_search(Arborescence &a1, Arborescence &a2, const Vertices &vertices, const Matrix &costs, const Penalties &penalties, int num_nodes, const Node &root_node, const Node &t, distance_t distance_limit_D, penalty_t &lambda_1, penalty_t &lambda_2, penalty_t &theta_1, penalty_t &theta_2)
{
    lambda_1 = lambda_2 = 0;
    for (Node v : vertices)
    {
        if (penalties[v] > EPS)
        {
            if (lambda_2 < costs[root_node][v] / penalties[v] + 1)
            {
                lambda_2 = costs[root_node][v] / penalties[v] + 1;
            }
        }
    }
    
    Matrix c1 = Matrix(2 * num_nodes, std::vector<distance_t>(2 * num_nodes, 0));
    Matrix c2 = Matrix(2 * num_nodes, std::vector<distance_t>(2 * num_nodes, 0));
    for (int row = 0; row < num_nodes; row++)
    {
        for (int col = 0; col < num_nodes; col++)
        {
            c1[row][col] = costs[row][col];
            c2[row][col] = costs[row][col];
        }
    }
    Vertices v1 = vertices, v2 = vertices;
    Penalties p1 = Penalties(2 * num_nodes, 0);
    Penalties p2 = Penalties(2 * num_nodes, 0);
    for (int cnt = 0; cnt < num_nodes; cnt++)
    {
        p1[cnt] = penalties[cnt];
        p2[cnt] = penalties[cnt];
    }
    for (int cnt = 0; cnt < penalties.size(); cnt++)
    {
        p1[cnt] *= lambda_1;
        p2[cnt] *= lambda_2;
    }
    p1[t] = 1 + costs[root_node][t];
    p2[t] = 1 + costs[root_node][t];
    theta_1 = 0, theta_2 = 0;

    a1 = iterPCA_with_check(v1, c1, p1, theta_1, num_nodes - 1, root_node);
    a2 = iterPCA_with_check(v2, c2, p2, theta_2, num_nodes - 1, root_node);
    binary_search_recursive(a1, a2, vertices, costs, lambda_1, lambda_2, theta_1, theta_2, penalties, num_nodes, root_node, t, distance_limit_D);
    assert(a1.find(t) != a1.end() && a2.find(t) != a2.end());
    // assert whether t is in a1 and a2
}

void recursive_dfs(Graph &g, const std::unordered_set<Node> &r_t_set, std::unordered_set<Node> &visited_so_far, Node top, Path &s_t_path)
{
    visited_so_far.insert(top);
    s_t_path.push_back(top);
    for (Node _next : g[top])
    {
        if (visited_so_far.find(_next) == visited_so_far.end() && r_t_set.find(_next) == r_t_set.end())
        {
            recursive_dfs(g, r_t_set, visited_so_far, _next, s_t_path);
            s_t_path.push_back(top);
        }
    }
}

void recursive_dfs_with_triangle_inequality(Graph &g, const std::unordered_set<Node> &r_t_set, std::unordered_set<Node> &visited_so_far, Node top, Path &s_t_path, const Node &t)
{
    visited_so_far.insert(top);
    if (top != t)
    {
        s_t_path.push_back(top);
    }
    for (Node _next : g[top])
    {
        if (visited_so_far.find(_next) == visited_so_far.end() && r_t_set.find(_next) == r_t_set.end())
        {
            recursive_dfs_with_triangle_inequality(g, r_t_set, visited_so_far, _next, s_t_path, t);
            
        }
    }
}

Path get_path(const Arborescence &arb_T, const Node &root_node, const Node &t, bool triangle_inequality)
{
    std::stack<Node> r_t_nodes;
    std::unordered_set<Node> r_t_set;
    Node tmp = t;
    Path s_t_path;
    Graph g;
    while (tmp != root_node)
    {
        r_t_nodes.push(tmp);
        r_t_set.insert(tmp);
        tmp = arb_T.at(tmp);
    }
    r_t_nodes.push(root_node);
    r_t_set.insert(root_node);
    
    // Turn arborescence into a graph
    for (std::pair<Node, Node> kv : arb_T)
    {
        Node _u = kv.second, _w = kv.first;
        g[_u].insert(_w);
    }
    while (!r_t_nodes.empty())
    {
        Node top = r_t_nodes.top();
        r_t_nodes.pop();
        // DFS
        // TODO can it be done without recursion?
        std::unordered_set<Node> visited_so_far;
        if (!triangle_inequality)
        {
            recursive_dfs(g, r_t_set, visited_so_far, top, s_t_path);
        }
        else
        {
            recursive_dfs_with_triangle_inequality(g, r_t_set, visited_so_far, top, s_t_path, t);
            if (top == t)
            {
                s_t_path.push_back(t);
            }
        }
    }
    return s_t_path;
}

penalty_t get_path_reward(const Path &p, const Rewards &rewards)
{
    penalty_t path_reward = 0;
    for (Node _ : p)
    {
        path_reward += rewards[_];
    }
    return path_reward;
}

distance_t get_path_distance(const Path &p, const Matrix &distances, const Node &root_node)
{
    distance_t path_distance = 0;
    Node previous_node = root_node;
    for (Node n : p)
    {
        path_distance += distances[previous_node][n];
        previous_node = n;
    }
    return path_distance;
}

Path get_best_path_dp(
    const Path &p,
    const Matrix &costs,
    const Rewards &rewards,
    const Node &root_node,
    distance_t distance_limit_D
    )
{
    std::vector<Node> visited_nodes;
    std::unordered_map<Node, Node> dp_previous_node;
    std::unordered_map<Node, reward_t> dp_reward;
    std::unordered_map<Node, distance_t> dp_distance;
    for (Node n: p){
        if (n == root_node) 
        {
            dp_previous_node[n] = root_node;
            dp_reward[n] = 0;
            dp_distance[n] = 0;
            continue;
        }
        reward_t max_reward = 0;
        distance_t max_distance = 4 * distance_limit_D;
        Node max_node = root_node;
        for (Node previous_node: visited_nodes){
            reward_t current_reward = dp_reward[previous_node] + rewards[n];
            distance_t current_distance = dp_distance[previous_node] + costs[previous_node][n];
            if (distance_limit_D - current_distance >= EPS){
                if (max_reward < current_reward){
                    max_node = previous_node;
                    max_reward = current_reward;
                    max_distance = current_distance;
                }
                else{
                    if (max_reward == current_reward && max_distance > current_distance){
                        max_node = previous_node;
                        max_reward = current_reward;
                        max_distance = current_distance;
                    }
                }
            }
        }
        dp_previous_node[n] = max_node;
        dp_reward[n] = max_reward;
        dp_distance[n] = max_distance;
        visited_nodes.append(n);
    }
    Node optimal_node = root_node;
    reward_t max_reward = 0;
    for (Node n: p){
        if (dp_distance[n] <= distance_limit_D && max_reward < dp_reward[n]){
            optimal_node = n;
            max_reward = dp_reward[n];
        }
    }
    // std::stack<Node> reverse_path;
    Path best_path;
    Node current_node = optimal_node;
    while (current_node != root_node){
        best_path.push_front(current_node);
        current_node = dp_previous_node[current_node];
    }
    best_path.push_front(root_node);
    // reverse_path.push(root_node);

    // 
    // while (!reverse_path.empty()){
    //     Node tmp = reverse_path.top();
    //     reverse_path.pop();
    //     best_path.push_back(tmp);
    // }
    return best_path;
    // dp_previous_node[root_node] = root_node;
    // dp_reward[root_node] = rewards[root_node];
    // dp_distance[root_node] = costs[root_node][root_node];



}

Path get_best_path(const Path &p, const Matrix &costs, const Rewards &rewards, const Node &root_node, distance_t distance_limit_D)
{
    Path best_path, p_i = p;
    reward_t best_path_reward = -1;

    distance_t current_path_distance = 0;
    reward_t current_path_reward = 0;

    Path current_path;
    Path::iterator initial_node_iterator = p_i.begin();
    
    if (std::next(p_i.begin()) == p_i.end())
        return p_i;
    

    // while (initial_node_iterator != p_i.end())
    for (initial_node_iterator++; initial_node_iterator != p_i.end(); ++initial_node_iterator)
    {
        current_path.clear();

        current_path.push_back(root_node);

        Node initial_node = *initial_node_iterator;

        #ifndef NDEBUG
        std::cout << "Initial Node " << initial_node << " ";
        #endif
        current_path.push_back(initial_node);
        current_path_distance = costs[root_node][initial_node];
        Node previous_node = initial_node;
        current_path_reward = rewards[initial_node];
        distance_t next_path_distance = current_path_distance;
        if (initial_node_iterator != p_i.end())
        {
            for (Path::iterator node_iterator = std::next(initial_node_iterator, 1); node_iterator != p_i.end(); ++node_iterator)
            {
                #ifndef NDEBUG
                std::cout << *node_iterator << " ";
                #endif
                next_path_distance += costs[previous_node][*node_iterator];

                if (next_path_distance > distance_limit_D)
                {
                    break;
                }
                current_path.push_back(*node_iterator);
                current_path_reward += rewards[*node_iterator];
                current_path_distance = next_path_distance;
                previous_node = *node_iterator;
            }
        }
        #ifndef NDEBUG
        std::cout << std::endl;
        #endif
        if (current_path_reward > best_path_reward)
        {
            best_path = current_path;
            best_path_reward = current_path_reward;
        }
    }
    return best_path;
    
}

Path get_best_path_between_the_two(const Arborescence &a1, const Arborescence &a2, const Matrix &costs, const Rewards &rewards, const Node &root_node, const Node &t, distance_t distance_limit_D)
{
    Path p1 = get_path(a1, root_node, t, true);
    Path p2 = get_path(a2, root_node, t, true);
    // Replace cut_path with get_best_path

    Path best_path = get_best_path_dp(p1, costs, rewards, root_node, distance_limit_D);

    Path tmp = get_best_path_dp(p2, costs, rewards, root_node, distance_limit_D);

    if (get_path_reward(tmp, rewards) > get_path_reward(best_path, rewards))
    {
        best_path = tmp;
    }
    #ifndef NDEBUG
    for (Node n: best_path){
        std::cout << n << ": " << rewards[n] << " ";
    }
    std::cout << std::endl;
    reward_t brute_force_reward = get_path_reward(best_path, rewards);

    Path tmp1 = cut_path(p1, costs, rewards, root_node, distance_limit_D);

    Path tmp2 = cut_path(p2, costs, rewards, root_node, distance_limit_D);
    reward_t old_alg_reward = std::max(get_path_reward(tmp1, rewards), get_path_reward(tmp2, rewards));
    std::cout << "Old algorithm reward: " << old_alg_reward << std::endl
              << "Brute force reward: " << brute_force_reward << std::endl;
    #endif
    
    return best_path;
}

Path cut_path(const Path &p, const Matrix &costs, const Rewards &rewards, const Node &root_node, distance_t distance_limit_D)
{
    Path best_path, p_i = p;
    reward_t best_path_reward = -1;

    distance_t current_path_distance = 0;
    reward_t current_path_reward = 0;
    // Path current_path;
    Node previous_node = root_node;
    Path current_path;
    current_path.push_back(root_node);
    p_i.pop_front();
    do
    {
        Node current_node = p_i.front();
        distance_t next_path_distance = current_path_distance + costs[previous_node][current_node];
        // TODO I don't think the second condition is necessary, but I put it there just for accuracy
        if (next_path_distance > distance_limit_D && current_path_distance <= distance_limit_D)
        {
            // TODO what happens if they are equal?
            if (current_path_reward > best_path_reward)
            {
                best_path = current_path;
                best_path_reward = current_path_reward;
            }
            previous_node = root_node;
            current_path = Path();
            current_path.push_back(root_node);
            // current_path_distance = costs[root_node][current_node];
            // current_path_reward = rewards[current_node];
            current_path_distance = 0;
            current_path_reward = 0;
        }
        else
        {
            current_path_distance += costs[previous_node][current_node];
            current_path_reward += rewards[current_node];
            current_path.push_back(current_node);
            previous_node = current_node;
            p_i.pop_front();
        }
    } while (!p_i.empty());
    if (current_path_reward > best_path_reward)
    {
        best_path = current_path;
        best_path_reward = current_path_reward;
    }
    return best_path;
}

Path orienteering(Vertices &vertices, const Node &root_node, Node &t, const Matrix &distances, const Rewards &rewards, distance_t distance_limit_D, int number_of_nodes, penalty_t &upper_bound)
{
    Arborescence a1, a2;
    penalty_t lambda_1, lambda_2;
    penalty_t R_t = 0;
    for (Node n : vertices)
    {
        if (n != root_node)
        {
            R_t += rewards[n];
        }
    }

    penalty_t theta_1, theta_2;
    binary_search(a1, a2, vertices, distances, rewards, number_of_nodes, root_node, t, distance_limit_D, lambda_1, lambda_2, theta_1, theta_2);
    penalty_t alpha = (distance_limit_D - edge_cost(a2, distances)) / (edge_cost(a1, distances) - edge_cost(a2, distances));
    upper_bound = R_t - (alpha * theta_1 + (1 - alpha) * theta_2 - distance_limit_D) / lambda_1;

    Path tmp = get_best_path_between_the_two(a1, a2, distances, rewards, root_node, t, distance_limit_D);
    
    return tmp;
}

// TODO this function may be parallelized
std::pair<Node, Path> orienteering(const Vertices &vertices, const Node &root_node, const Matrix &distances, const Rewards &rewards, distance_t distance_limit_D, std::unordered_map<Node, penalty_t> &upper_bounds)
{
    penalty_t best_path_reward = -1, upper_bound;
    // penalty_t best_upper;
    Path best_path;
    Node best_t = root_node;
    for (Node t : vertices)
    {
        #ifndef NDEBUG
        std::cout << "Current Node: " << t << std::endl;
        #endif
        if (t == root_node || distances[root_node][t] > distance_limit_D)
            continue;
        Arborescence a1, a2;
        // TODO why do I have number_of_nodes?
        Vertices v_copy;
        // node_list maps new nodes to the original ones
        std::vector<Node> node_list;
        // node_map maps original nodes to new ones
        std::unordered_map<Node, Node> node_map;
        v_copy.insert(root_node);
        node_list.push_back(root_node);
        node_map[root_node] = root_node;
        for (Node _ : vertices)
        {
            if (root_node != _ && distances[root_node][_] <= distances[root_node][t])
            {
                // v_copy.insert(_);
                node_list.push_back(_);
                v_copy.insert(node_list.size() - 1);
                node_map[_] = node_list.size() - 1;
            }
        }
        // TODO Compute new distances and rewards
        int num_nodes = node_list.size();
        Matrix new_distances = Matrix(num_nodes, std::vector<distance_t>(num_nodes, 0));
        Penalties new_rewards = Rewards(num_nodes);
        for (int i = 0; i < num_nodes; ++i)
        {
            new_rewards[i] = rewards[node_list[i]];
            for (int j = 0; j < num_nodes; ++j)
            {
                new_distances[i][j] = distances[node_list[i]][node_list[j]];
            }
        }

        Path new_tmp = orienteering(v_copy, root_node, node_map[t], new_distances, new_rewards, distance_limit_D, num_nodes, upper_bound);

        upper_bounds[t] = upper_bound;
        Path tmp;
        #ifndef NDEBUG
        std::cout << rewards.size() << std::endl;
        #endif

        #ifndef NDEBUG
            for (Node n: node_list){
                std::cout << n << " ";
            }
            std::cout << std::endl;
        #endif
        for (Node _ : new_tmp)
        {
            #ifndef NDEBUG
            std::cout << _ << " "; 
            #endif
            
            // std::cout <<node_list[_] << " ";
            tmp.push_back(node_list[_]);
        }
        // exit(1);
        
        // Mapping complete
        penalty_t path_reward = get_path_reward(tmp, rewards);
        
        if (best_path_reward < path_reward)
        {
            best_path_reward = path_reward;
            best_path = tmp;
            best_t = t;
        }
    }
    
    return std::pair<Node, Path>(best_t, best_path);
}
