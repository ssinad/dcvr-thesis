#ifndef __S_T_PATH__
#define __S_T_PATH__
#include <list>
#include "Node.hpp"
#include "Penalties.h"
#include "Matrix.h"

using Path = std::list<Node>;

struct PathWrapper
{
  Path path;
  double value;
  bool is_basic = false;
};

penalty_t get_path_reward(const Path &p, const Rewards &rewards);
distance_t get_path_distance(const Path &p, const Matrix &distances);

#endif