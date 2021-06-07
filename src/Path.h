#ifndef __S_T_PATH__
#define __S_T_PATH__
#include <list>
#include "Node.hpp"

using Path = std::list<Node>;

struct PathWrapper
{
  Path path;
  double value;
  bool is_basic = false;
};
#endif