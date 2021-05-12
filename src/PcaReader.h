#ifndef __PCAREADER__
#define __PCAREADER__
#include <string>
#include "Matrix.h"
#include "Penalties.h"
#include "Vertices.h"

class PcaReader
{
  private:
    int num_nodes, matrix_size;
    Matrix matrix;
    Vertices vertices;
    Penalties penalties;

  public:
    PcaReader(std::string file_name);

    Matrix get_matrix();

    Vertices get_vertices();

    Penalties get_penalties();

    Penalties get_penalties(penalty_t penalty);

    int get_num_nodes();
};
#endif
