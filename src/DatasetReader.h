#include <string>
#include "Matrix.h"
#include "Penalties.h"
#include "Node.hpp"
#include "Vertices.h"

enum Dataset_type {DVRP, ORIENTEERING, P2P};

class DatasetReader
{
private:
  Matrix distances;
  Rewards rewards;
  Vertices vertices;
  Node root_node = 0;
  Node start_node, finish_node;
  int num_nodes = 0;

public:
  void read_file(std::string filename, Dataset_type read_rewards=Dataset_type::DVRP);
  Matrix get_matrix();
  Penalties get_penalties();
  Vertices get_vertices();
  Node get_root_node();
  Node get_start_node();
  Node get_finish_node();
};

