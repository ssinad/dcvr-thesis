#ifndef __BINARY_ARB_SEARCH__
#define __BINARY_ARB_SEARCH__
#include "Arborescence.h"
#include "Matrix.h"
#include "Vertices.h"
#include "Penalties.h"
#include "Path.h"

Path get_path(const Arborescence &arb_T, const Node &root_node, const Node &t, bool triangle_inequality = true);
distance_t edge_cost(const Arborescence &arb_T, const Matrix &costs);
void binary_search(Arborescence &a1, Arborescence &a2, const Vertices &vertices, const Matrix &costs, const Penalties& penalties, int num_nodes, const Node &t, distance_t distance_limit_D);
Path cut_path(const Path &p_i, const Matrix &costs, const Rewards &rewards, const Node &root_node, distance_t distance_limit_D);
std::pair<Node, Path> orienteering(const Vertices &vertices, const Node &root_node, const Matrix &distances, const Rewards &rewards, distance_t distance_limit_D, std::unordered_map<Node, penalty_t>& upper_bounds);
penalty_t get_path_reward(const Path &p, const Rewards &rewards);
distance_t get_path_distance(const Path &p, const Matrix &distances, const Node& root_node);
#endif