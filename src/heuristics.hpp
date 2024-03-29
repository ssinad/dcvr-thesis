#ifndef __HEURISTICS__
#define __HEURISTICS__

#include "Path.h"
#include "Penalties.h"
#include "Matrix.h"
#include "Vertices.h"


Path post_process(Path &path, const Rewards &rewards, const Matrix &distances, const Node &root_node, const distance_t distance_limit_D);
Path cycle_post_process(Path &path, const Rewards &rewards, const Matrix &distances, const Node &root_node, const distance_t distance_limit_D);
std::pair<bool, Path> path_generation_heuristic_1(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node);
std::pair<bool, Path> path_generation_heuristic_2(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node);
std::pair<bool, Path> path_generation_heuristic_3(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node);
std::pair<bool, Path> cycle_generation_heuristic_1(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node);
std::pair<bool, Path> cycle_generation_heuristic_2(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node);
std::pair<bool, Path> cycle_generation_heuristic_3(const Vertices &vertices, const Rewards &rewards, const Matrix &distances, const distance_t distance_limit_D, const Node &root_node);

#endif
