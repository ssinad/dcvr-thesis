#include <string>
#include "Matrix.h"
#include "Penalties.h"
#include "Node.hpp"
#include "Vertices.h"

class DatasetReader
{
private:
  Matrix distances;
  Rewards rewards;
  Vertices vertices;
  Node root_node = 0;
  int num_nodes = 0;

public:
  void read_file(std::string filename, bool read_rewards=false);
  Matrix get_matrix();
  Penalties get_penalties();
  Vertices get_vertices();
  Node get_root_node();
};