#ifndef __BINARY_ARB_SEARCH__
#define __BINARY_ARB_SEARCH__
#include "Arborescence.h"
#include "Matrix.h"
#include "Vertices.h"
#include "Penalties.h"
#include "Path.h"
#include <map>
#include <chrono>

using namespace std::chrono;

const double LAMBDA_EPSILON = 1e-6;

struct BestPathInfo{
    duration<double> running_time;
    penalty_t upper_bound = 0;
    penalty_t lambda;
    penalty_t theta;
    distance_t arb_distance;
    reward_t arb_reward;
    reward_t r_t_path_reward;
    distance_t path_distance;
    distance_t r_t_path_distance;
    reward_t path_reward = 0;
    Path path;
    // reward_t a1_reward;
    // reward_t a2_reward;
    // distance_t a1_cost;
    // distance_t a2_cost;
};

struct BoundInfo{
    duration<double> running_time;
    penalty_t upper_bound = 0;
    penalty_t lambda;
    penalty_t theta;
    distance_t arb_distance;
    reward_t arb_reward;
    distance_t path_distance;
    reward_t path_reward = 0;
};

using LambdaMapping = std::map<penalty_t, BoundInfo>;

// Path get_path(  
//                 const Arborescence &arb_T,
//                 const Node &root_node,
//                 const Node &t,
//                 bool triangle_inequality = true
//             );
// distance_t edge_cost(const Arborescence &arb_T, const Matrix &costs);




// void binary_search(
                //     Arborescence &a1,
                //     Arborescence &a2,
                //     const Vertices &vertices,
                //     const Matrix &costs,
                //     const Penalties& penalties,
                //     int num_nodes,
                //     const Node &t,
                //     distance_t distance_limit_D
                // );
// Path cut_path(const Path &p_i, const Matrix &costs, const Rewards &rewards, const Node &root_node, distance_t distance_limit_D);
std::pair<Node, Path> rooted_orienteering(
                                    const Vertices &,
                                    const Node &,
                                    const Matrix &,
                                    const Rewards &,
                                    distance_t,
                                    std::unordered_map<Node, BestPathInfo>&,
                                    std::unordered_map<Node, BoundInfo>&
                                );

std::pair<Node, Path> cycle_orienteering(
                                    const Vertices &,
                                    const Node &,
                                    const Matrix &,
                                    const Rewards &,
                                    distance_t,
                                    std::unordered_map<Node, BestPathInfo>&,
                                    std::unordered_map<Node, BoundInfo>&
                                );

std::pair<Node, Path> p2p_orienteering(
    const Vertices &vertices,
    const Node &start_node,
    const Node &finish_node,
    const Matrix &distances,
    const Rewards &rewards,
    distance_t distance_limit_D, 
    std::unordered_map<Node, BestPathInfo> &best_path_info_map,
    std::unordered_map<Node, BoundInfo> &best_bound_info_map
    );

#endif