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
    // GraphReader(std::string file_name, penalty_t lambda)
    // : _lambda(lambda)
    // {
    //     std::ifstream input_file;
    //     int num_edges;
    //     input_file.open(file_name);
    //     input_file >> num_nodes >> num_edges;

    //     matrix = Matrix(num_nodes, std::vector<distance_t>(num_nodes, 0));
    //     for (int cnt = 0; cnt < num_edges; cnt++)
    //     {
    //         Node u, v;
    //         distance_t dist;
    //         input_file >> u >> v >> dist;
    //         matrix[u][v] = dist;
    //     }

    //     input_file.close();
    // }

    void write_to_file(std::string file_name);
    // void write_file(std::string file_name){
    //     std::ofstream output_file;
    //     output_file.open(file_name);
    //     output_file << num_nodes << std::endl;
    //     for (int row = 0; row < num_nodes; row++){
    //         for (int col = 0; col < num_nodes; col++){
    //             output_file << matrix[row][col] << " ";
    //         }
    //         output_file << std::endl;
    //     }
    //     for (int row = 0; row < num_nodes; row++){
    //         output_file << _lambda << " " ;
    //     }
    //     output_file << std::endl;
    //     output_file.close();
    // }

    Matrix get_matrix();
    // Matrix& get_matrix(){
    //     return matrix;
    // }

    Penalties get_penalties();
    // Penalties& get_penalties()
    // {
    //     Penalties penalties(num_nodes, _lambda);
    // }
    void apsp();
};
#endif