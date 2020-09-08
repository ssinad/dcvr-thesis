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
    // {
    //     std::ifstream input_file;
    //     // input_file = std::ifstream();
    //     input_file.open(file_name);

    //     if (!input_file.good()) {
    //         std::cout << "Cannot open " << file_name << "\n";
    //         exit(1);
    //     }

    //     input_file >> num_nodes;
    //     matrix_size = 2 * num_nodes;
    //     penalties = Penalties(matrix_size);
    //     matrix = Matrix(matrix_size, std::vector<distance_t>(matrix_size, 0));
    //     for (int row = 0; row < num_nodes; row++){
    //         for (int col = 0; col < num_nodes; col++){
    //             input_file >> matrix[row][col];
    //         }
    //     }
    //     for (int cnt = 0; cnt < num_nodes; ++cnt){
    //         vertices.insert(cnt);
    //         input_file >> penalties[cnt];
    //     }
    //     input_file.close();

    // }

    Matrix get_matrix();
    // {
    //     return matrix;
    // }

    Vertices get_vertices();
    // {
    //     return vertices;
    // }

    Penalties get_penalties();
    // {
    //     return penalties;
    // }

    Penalties get_penalties(penalty_t penalty);
    // {
    //     Penalties _ = Penalties(matrix_size, penalty);
    //     _[root_node] = 0;
    //     return _;
    // }

    int get_num_nodes();
    // {
    //     return num_nodes;
    // }
};
#endif
