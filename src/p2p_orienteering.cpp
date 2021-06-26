#include "orienteering.h"
#include "iterPCA.hpp"
#include <stack>
#include <unordered_set>
#include "Our_Graph.h"
#include <assert.h>
#include <map>
#include <iostream>
#include <cmath>
#include <algorithm>

using FeasiblePathExtractor = Path (&)(
    const Path &,
    const Matrix &,
    const Rewards &,
    distance_t,
    const Node &,
    const Node &
    );

static distance_t limit_D; // Not recommended

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

void recursive_dfs(
    Graph &g,
    const std::unordered_set<Node> &r_t_set,
    std::unordered_set<Node> &visited_so_far,
    Node top,
    Path &s_t_path
    )
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

void recursive_dfs_with_triangle_inequality(
    Graph &g,
    const std::unordered_set<Node> &r_t_set,
    std::unordered_set<Node> &visited_so_far,
    Node top,
    Path &s_t_path,
    const Node &furthest_node_guess
    )
{
    visited_so_far.insert(top);
    if (top != furthest_node_guess)
    {
        s_t_path.push_back(top);
    }
    for (Node _next : g[top])
    {
        if (visited_so_far.find(_next) == visited_so_far.end() && r_t_set.find(_next) == r_t_set.end())
        {
            recursive_dfs_with_triangle_inequality(g, r_t_set, visited_so_far, _next, s_t_path, furthest_node_guess);
            
        }
    }
}

Path get_path(const Arborescence &arb_T, const Node &root_node, const Node &furthest_node_guess, bool triangle_inequality, Path &r_t_path)
{
    std::stack<Node> r_t_nodes;
    std::unordered_set<Node> r_t_set;
    Node tmp = furthest_node_guess;
    Path s_t_path;
    Graph g;
    r_t_path.clear();
    while (tmp != root_node)
    {
        r_t_nodes.push(tmp);
        r_t_set.insert(tmp);
        tmp = arb_T.at(tmp);
        r_t_path.push_front(tmp);
    }
    r_t_nodes.push(root_node);
    r_t_set.insert(root_node);
    r_t_path.push_front(root_node);
    
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
            recursive_dfs_with_triangle_inequality(g, r_t_set, visited_so_far, top, s_t_path, furthest_node_guess);
            if (top == furthest_node_guess)
            {
                s_t_path.push_back(furthest_node_guess);
            }
        }
    }
    assert(*(s_t_path.begin()) == root_node);
    return s_t_path;
}

void binary_search_recursive(
    Arborescence &a1,
    Arborescence &a2,
    const Vertices &vertices,
    const Matrix &costs,
    penalty_t &lambda_1,
    penalty_t &lambda_2,
    // penalty_t &theta_1,
    // penalty_t &theta_2,
    const Penalties &penalties,
    int num_nodes,
    const Node &root_node,
    const Node &furthest_node_guess,
    distance_t distance_limit_D,
    BestPathInfo &best_path_info,
    LambdaMapping &best_bound_info,
    FeasiblePathExtractor get_feasible_path,
    const Node &start_node,
    const Node &finish_node
    )
{
    if (lambda_1 + LAMBDA_EPSILON >= lambda_2)
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
    p1[furthest_node_guess] = 1 + costs[root_node][furthest_node_guess];
    assert( costs[root_node][furthest_node_guess] <= distance_limit_D + DISTANCE_EPSILON);
    penalty_t theta = 0; //, theta_1, theta_2;
    Path r_t_path;

    Arborescence a = iterPCA_with_check(v1, c1, p1, theta, num_nodes - 1, root_node);
    Path tmp = get_path(a, root_node, furthest_node_guess, true, r_t_path);
    Path reverse_tmp;
    reverse_tmp.clear();
    if (root_node == finish_node){
        for (Node n: tmp){
            reverse_tmp.push_front(n);
        }
    }
    else{
        reverse_tmp = tmp;
    }
    Path tmp_p = get_feasible_path(reverse_tmp, costs, penalties, limit_D, start_node, finish_node);
    
    BoundInfo tmp_bound_info;
    tmp_bound_info.arb_distance = edge_cost(a, costs);
    tmp_bound_info.arb_reward = total_reward(a, penalties);
    tmp_bound_info.path_distance = get_path_distance(tmp_p, costs);
    tmp_bound_info.path_reward = get_path_reward(tmp_p, penalties);
    tmp_bound_info.theta = theta;
    best_bound_info[lambda] = tmp_bound_info;

    if (get_path_reward(tmp_p, penalties) > get_path_reward(best_path_info.path, penalties)){
        // best_path_info.upper_bound = tmp_bound_info;
        best_path_info.arb_distance = edge_cost(a, costs);
        best_path_info.arb_reward = total_reward(a, penalties);
        best_path_info.path_distance = get_path_distance(tmp_p, costs);
        best_path_info.path_reward = get_path_reward(tmp_p, penalties);
        best_path_info.r_t_path_reward = get_path_reward(r_t_path, penalties);
        best_path_info.r_t_path_distance = get_path_distance(r_t_path, costs);
        best_path_info.path = tmp_p;
    }

    distance_t tree_cost = edge_cost(a, costs);
    if (tree_cost - distance_limit_D <= DISTANCE_EPSILON && tree_cost - distance_limit_D >= -DISTANCE_EPSILON)
    {
        // theta_1 = theta;
        // theta_2 = theta;
        lambda_1 = lambda_2 = lambda;
        a1 = a;
        a2 = a;
        return;
    }
    else
    {
        if (tree_cost + DISTANCE_EPSILON > distance_limit_D)
        {
            a2 = a;
            // theta_2 = theta;
            lambda_2 = lambda;
            // TODO Should this be the original penalties, costs, vertices?
            binary_search_recursive(a1, a2, vertices, costs, lambda_1, lambda_2, penalties, num_nodes, root_node, furthest_node_guess, distance_limit_D, best_path_info, best_bound_info, get_feasible_path, start_node, finish_node);
            return;
        }
        else
        {
            a1 = a;
            // theta_1 = theta;
            lambda_1 = lambda;
            binary_search_recursive(a1, a2, vertices, costs, lambda_1, lambda_2, penalties, num_nodes, root_node, furthest_node_guess, distance_limit_D, best_path_info, best_bound_info, get_feasible_path, start_node, finish_node);
            return;
        }
    }
}

void binary_search(
    Arborescence &a1,
    Arborescence &a2,
    const Vertices &vertices,
    const Matrix &costs,
    const Penalties &penalties,
    int num_nodes,
    const Node &root_node,
    const Node &furthest_node_guess,
    distance_t distance_limit_D,
    penalty_t &lambda_1,
    penalty_t &lambda_2,
    BestPathInfo &best_path_info,
    LambdaMapping &best_bound_info,
    FeasiblePathExtractor get_feasible_path,
    const Node &start_node,
    const Node &finish_node
    )
{
    lambda_1 = lambda_2 = 0;
    for (Node v : vertices)
    {
        if (penalties[v] > REWARD_EPSILON)
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
    p1[furthest_node_guess] = 1 + costs[root_node][furthest_node_guess];
    p2[furthest_node_guess] = 1 + costs[root_node][furthest_node_guess];
    penalty_t theta_1 = 0, theta_2 = 0;

    
    BoundInfo tmp_bound_info_1, tmp_bound_info_2;
    Path r_t_path;
    a1 = iterPCA_with_check(v1, c1, p1, theta_1, num_nodes - 1, root_node);
    Path tmp = get_path(a1, root_node, furthest_node_guess, true, r_t_path);
    // for (Node n: tmp){
    //     std::clog << n << " ";
    // }
    // std::clog << std::endl;
    Path reverse_tmp;
    reverse_tmp.clear();
    if (root_node == finish_node){
        for (Node n: tmp){
            reverse_tmp.push_front(n);
        }
    }
    else{
        reverse_tmp = tmp;
    }
    Path tmp_p = get_feasible_path(reverse_tmp, costs, penalties, limit_D, start_node, finish_node);
    
    tmp_bound_info_1.arb_distance = edge_cost(a1, costs);
    tmp_bound_info_1.arb_reward = total_reward(a1, penalties);
    tmp_bound_info_1.path_distance = get_path_distance(tmp_p, costs);
    tmp_bound_info_1.path_reward = get_path_reward(tmp_p, penalties);
    tmp_bound_info_1.theta = theta_1;
    best_bound_info[lambda_1] = tmp_bound_info_1;

    if (get_path_reward(tmp_p, penalties) > get_path_reward(best_path_info.path, penalties)){
        // best_path_info.upper_bound = tmp_bound;
        best_path_info.arb_distance = edge_cost(a1, costs);
        best_path_info.arb_reward = total_reward(a1, penalties);
        best_path_info.path_distance = get_path_distance(tmp_p, costs);
        best_path_info.path_reward = get_path_reward(tmp_p, penalties);
        best_path_info.r_t_path_reward = get_path_reward(r_t_path, penalties);
        best_path_info.r_t_path_distance = get_path_distance(r_t_path, costs);
        best_path_info.path = tmp_p;
    }


    a2 = iterPCA_with_check(v2, c2, p2, theta_2, num_nodes - 1, root_node);
    // tmp_bound = theta_2 / lambda_2;
    tmp.clear();
    tmp = get_path(a2, root_node, furthest_node_guess, true, r_t_path);
    // for (Node n: tmp){
    //     std::clog << n << " ";
    // }
    // std::clog << std::endl;
    reverse_tmp.clear();
    if (root_node == finish_node){
        for (Node n: tmp){
            reverse_tmp.push_front(n);
        }
    }
    else{
        reverse_tmp = tmp;
    }
    tmp_p = get_feasible_path(reverse_tmp, costs, penalties, limit_D, start_node, finish_node);

    tmp_bound_info_2.arb_distance = edge_cost(a2, costs);
    tmp_bound_info_2.arb_reward = total_reward(a2, penalties);
    tmp_bound_info_2.path_distance = get_path_distance(tmp_p, costs);
    tmp_bound_info_2.path_reward = get_path_reward(tmp_p, penalties);
    tmp_bound_info_2.theta = theta_2;
    best_bound_info[lambda_2] = tmp_bound_info_2;
    if (get_path_reward(tmp_p, penalties) > get_path_reward(best_path_info.path, penalties)){
        // best_path_info.upper_bound = tmp_bound;
        best_path_info.arb_distance = edge_cost(a2, costs);
        best_path_info.arb_reward = total_reward(a2, penalties);
        best_path_info.path_distance = get_path_distance(tmp_p, costs);
        best_path_info.path_reward = get_path_reward(tmp_p, penalties);
        best_path_info.r_t_path_reward = get_path_reward(r_t_path, penalties);
        best_path_info.r_t_path_distance = get_path_distance(r_t_path, costs);
        best_path_info.path = tmp_p;
    }

    if (edge_cost(a2, costs) > distance_limit_D + DISTANCE_EPSILON){
        binary_search_recursive(a1, a2, vertices, costs, lambda_1, lambda_2, penalties, num_nodes, root_node, furthest_node_guess, distance_limit_D, best_path_info, best_bound_info, get_feasible_path, start_node, finish_node);
    }
    else
    {
        // std::clog << costs[root_node][furthest_node_guess] << "," << edge_cost(a2, costs) << ", " << distance_limit_D << std::endl;
    }
    assert(a1.find(furthest_node_guess) != a1.end() && a2.find(furthest_node_guess) != a2.end());
    // assert whether t is in a1 and a2
}

Path get_best_path(
    const Path &p,
    const Matrix &costs,
    const Rewards &rewards,
    distance_t distance_limit_D,
    const Node &start_node,
    const Node &finish_node
    )
{
    Path best_path, p_i = p;
    reward_t best_path_reward = -1;

    for (Node n: p_i){
        std::clog << n << " ";
    }
    std::clog << std::endl;
    
    if (std::next(p_i.begin()) == p_i.end())
        return p_i;
    
    assert (*(p_i.begin()) != finish_node);
    assert (*(p_i.rbegin()) != start_node);
    
    if (*(p_i.begin()) == start_node){
        p_i.pop_front();
    }

    if (*(p_i.rbegin()) == finish_node){
        p_i.pop_back();
    }
    
    distance_t current_path_distance = 0;
    reward_t current_path_reward = 0;
    Node previous_node = start_node;

    Path current_path;
    Path::iterator initial_node_iterator = p_i.begin(), node_iterator;
    for (initial_node_iterator = p_i.begin(); initial_node_iterator != p_i.end(); initial_node_iterator++){
        current_path.clear();
        current_path_distance = 0;
        current_path_reward = rewards[start_node] + rewards[finish_node];
        previous_node = start_node;
        for (node_iterator = initial_node_iterator; node_iterator != p_i.end(); ++node_iterator){
            Node current_node = *node_iterator;
            current_path_distance += costs[previous_node][current_node];
            current_path_reward += rewards[current_node];
            current_path.push_back(current_node);
            if (current_path_distance + costs[current_node][finish_node] <= DISTANCE_EPSILON + distance_limit_D){
                if (current_path_reward > best_path_reward){
                    best_path = current_path;
                    best_path_reward = current_path_reward;
                }
            }
            previous_node = current_node;
        }
    }
    best_path.push_front(start_node);
    best_path.push_back(finish_node);
    
    assert(get_path_distance(best_path, costs) <= distance_limit_D + DISTANCE_EPSILON);
    // std :: clog << get_path_distance(best_path, costs) << ", " << distance_limit_D << std::endl;
    // best_path.push_back(root_node);
    assert(*(best_path.begin()) == start_node);
    assert(*(best_path.rbegin()) == finish_node);
    return best_path;
    
}


Path p2p_orienteering_with_guess(
    Vertices &vertices,
    const Node &root_node,
    Node &furthest_node_guess,
    const Matrix &distances,
    const Rewards &rewards,
    distance_t distance_limit_D,
    int number_of_nodes,
    FeasiblePathExtractor get_feasible_path,
    BestPathInfo &best_path_info,
    LambdaMapping &best_bound_info,
    const Node &start_node,
    const Node &finish_node
    )
{
    Arborescence a1, a2;
    penalty_t lambda_1, lambda_2;
    
    binary_search(a1, a2, vertices, distances, rewards, number_of_nodes, root_node, furthest_node_guess, distance_limit_D, lambda_1, lambda_2, best_path_info, best_bound_info, get_feasible_path, start_node, finish_node);
    
    return best_path_info.path;
}

// TODO this function may be parallelized
std::pair<Node, Path> p2p_orienteering(
    const Vertices &vertices,
    const Node &start_node,
    const Node &finish_node,
    const Matrix &distances,
    const Rewards &rewards,
    distance_t distance_limit_D, 
    std::unordered_map<Node, BestPathInfo> &best_path_info_map,
    std::unordered_map<Node, BoundInfo> &best_bound_info_map
    )
{
    limit_D = distance_limit_D;
    penalty_t best_path_reward = -1, upper_bound;
    // penalty_t best_upper;
    Path best_path;
    Node best_t = start_node;
    for (Node furthest_node_guess : vertices)
    {
        // #ifndef NDEBUG
        // std::clog << "Current Node: " << furthest_node_guess << std::endl;
        // #endif
        if (furthest_node_guess == start_node || furthest_node_guess == finish_node || distances[start_node][furthest_node_guess] + distances[furthest_node_guess][finish_node] > distance_limit_D + DISTANCE_EPSILON)
            continue;
        Arborescence a1, a2;
        // TODO why do I have number_of_nodes?
        Vertices v_copy;
        // node_list maps new nodes to the original ones
        std::vector<Node> node_list;
        // node_map maps original nodes to new ones
        std::unordered_map<Node, Node> node_map;
        v_copy.clear();
        // v_copy.insert(start_node);
        // v_copy.insert(finish_node);
        node_list.clear();
        node_list.push_back(start_node);
        node_map[start_node] = node_list.size() - 1;
        v_copy.insert(node_list.size() - 1);
        node_list.push_back(finish_node);
        node_map[finish_node] = node_list.size() - 1;
        v_copy.insert(node_list.size() - 1);
        for (Node _ : vertices)
        {
            if (start_node != _ && finish_node != _ && distances[start_node][_] + distances[_][finish_node] <= distances[start_node][furthest_node_guess] + distances[furthest_node_guess][finish_node] + DISTANCE_EPSILON)
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
        penalty_t sum_rewards = 0;
        for (int i = 0; i < num_nodes; ++i)
        {
            new_rewards[i] = rewards[node_list[i]];
            sum_rewards += new_rewards[i];
            for (int j = 0; j < num_nodes; ++j)
            {
                new_distances[i][j] = distances[node_list[i]][node_list[j]];
            }
        }

        // #ifndef NDEBUG
        //     std::clog << "Running orienteering with guess: " << furthest_node_guess << std::endl;
        // #endif
        
        LambdaMapping tmp_bound_1, tmp_bound_2;
        tmp_bound_1.clear();
        tmp_bound_2.clear();
        BestPathInfo best_path_info;
        BoundInfo best_bound_info;
        penalty_t upper_bound = sum_rewards, partial_upper_bound_1 = sum_rewards, partial_upper_bound_2 = sum_rewards;

        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        Path new_tmp_1 = p2p_orienteering_with_guess(v_copy, node_map[start_node], node_map[furthest_node_guess], new_distances, new_rewards, distance_limit_D - distances[furthest_node_guess][finish_node], num_nodes, get_best_path, best_path_info, tmp_bound_1, node_map[start_node], node_map[finish_node]);


        Path new_tmp_2 = p2p_orienteering_with_guess(v_copy, node_map[finish_node], node_map[furthest_node_guess], new_distances, new_rewards, distance_limit_D - distances[start_node][furthest_node_guess], num_nodes, get_best_path, best_path_info, tmp_bound_2, node_map[start_node], node_map[finish_node]);
        
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        
        std::clog << "{" << std::endl;
        for (auto& kv: tmp_bound_1){
            penalty_t lambda = kv.first;
            // upper_bound = std::min(upper_bound, 2 * (sum_rewards + (distance_limit_D / 2 - kv.second.theta) / lambda));
            // upper_bound = std::min(upper_bound, 2 * (sum_rewards + (distance_limit_D - distances[root_node][furthest_node_guess] - kv.second.theta) / lambda));
            partial_upper_bound_1 = std::min(partial_upper_bound_1, sum_rewards + (distance_limit_D - distances[furthest_node_guess][finish_node] - kv.second.theta) / lambda);
            upper_bound = std::min(upper_bound, sum_rewards + (distance_limit_D - kv.second.theta) / lambda);
            std::clog << lambda << " : " << kv.second.theta << " ," << std::endl;
        }
        std::clog << "}," << std::endl << "{" << std::endl;

        for (auto& kv: tmp_bound_2){
            penalty_t lambda = kv.first;
            // upper_bound = std::min(upper_bound, 2 * (sum_rewards + (distance_limit_D - distances[root_node][furthest_node_guess] - kv.second.theta) / lambda));
            partial_upper_bound_2 = std::min(partial_upper_bound_2, sum_rewards + (distance_limit_D - distances[start_node][furthest_node_guess] - kv.second.theta) / lambda);
            upper_bound = std::min(upper_bound, sum_rewards + (distance_limit_D - kv.second.theta) / lambda);
            std::clog << lambda << " : " << kv.second.theta << " ," << std::endl;
        }
        std::clog << "}" << std::endl;
        upper_bound = std::min(upper_bound, partial_upper_bound_1 + partial_upper_bound_2);

        duration<double> time_span = duration_cast<duration<double> >(t2 - t1);
        // #ifndef NDEBUG
        //     std::clog << time_span.count() * 1000.0 << " ms"<< std::endl;
        // #endif
        
        best_bound_info.upper_bound = upper_bound;
        best_bound_info.running_time = time_span;
        best_path_info.running_time = time_span;
        best_bound_info_map[furthest_node_guess] = best_bound_info;
        best_path_info_map[furthest_node_guess] = best_path_info;

        Path tmp_1, tmp_2;
        // #ifndef NDEBUG
        // std::clog << rewards.size() << std::endl;
        // #endif

        // #ifndef NDEBUG
        //     for (Node n: node_list){
        //         std::clog << n << " ";
        //     }
        //     std::clog << std::endl;
        // #endif
        for (Node _ : new_tmp_1)
        {
            tmp_1.push_back(node_list[_]);
        }
        for (Node _ : new_tmp_2)
        {
            tmp_2.push_back(node_list[_]);
        }
        // #ifndef NDEBUG
        //     std::clog << std::endl;
        // #endif
        // exit(1);
        
        // Mapping complete
        penalty_t path_reward = get_path_reward(tmp_1, rewards);
        
        if (best_path_reward < path_reward)
        {
            best_path_reward = path_reward;
            best_path = tmp_1;
            best_t = furthest_node_guess;
        }

        path_reward = get_path_reward(tmp_2, rewards);
        if (best_path_reward < path_reward)
        {
            best_path_reward = path_reward;
            best_path = tmp_2;
            best_t = furthest_node_guess;
        }

    }
    
    return std::pair<Node, Path>(best_t, best_path);
}
