#ifndef __GRAPHREADER__
#define __GRAPHREADER__
#include "Matrix.h"
#include "Vertices.h"
#include "Penalties.h"
#include <string>
// #include <fstream>

class GraphReader
{
  private:
    int num_nodes, num_edges;
    Matrix matrix, apsp_matrix;
    Vertices vertices;
    Penalties penalties;
    penalty_t _lambda;

  public:
    GraphReader(std::string file_name, penalty_t lambda);
  
    void write_to_file(std::string file_name);

    Matrix get_matrix();

    Penalties get_penalties();
    
    void apsp();
};
#endif