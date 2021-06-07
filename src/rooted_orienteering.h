#ifndef __BINARY_ARB_SEARCH__
#define __BINARY_ARB_SEARCH__
#include "Arborescence.h"
#include "Matrix.h"
#include "Vertices.h"
#include "Penalties.h"
#include "Path.h"
#include <chrono>

using namespace std::chrono;

const double LAMBDA_EPSILON = 1e-6;

struct OrienteeringInfo{
    duration<double> running_time;
    penalty_t upper_bound;
    reward_t a1_reward;
    reward_t a2_reward;
    distance_t a1_cost;
    distance_t a2_cost;
};

Path get_path(  
                const Arborescence &arb_T,
                const Node &root_node,
                const Node &t,
                bool triangle_inequality = true
            );
distance_t edge_cost(const Arborescence &arb_T, const Matrix &costs);
void binary_search(
                    Arborescence &a1,
                    Arborescence &a2,
                    const Vertices &vertices,
                    const Matrix &costs,
                    const Penalties& penalties,
                    int num_nodes,
                    const Node &t,
                    distance_t distance_limit_D
                );
Path cut_path(const Path &p_i, const Matrix &costs, const Rewards &rewards, const Node &root_node, distance_t distance_limit_D);
std::pair<Node, Path> rooted_orienteering(
                                    const Vertices &,
                                    const Node &,
                                    const Matrix &,
                                    const Rewards &,
                                    distance_t,
                                    std::unordered_map<Node, OrienteeringInfo>&
                                );
penalty_t get_path_reward(const Path &p, const Rewards &rewards);
distance_t get_path_distance(const Path &p, const Matrix &distances, const Node& root_node);


#endif