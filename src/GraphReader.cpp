#include "GraphReader.h"
#include <fstream>
#include <iostream>

GraphReader::GraphReader(std::string file_name, penalty_t lambda)
    : _lambda(lambda)
{
    std::ifstream input_file;
    int num_edges;
    input_file.open(file_name);

    input_file >> num_nodes >> num_edges;

    // TODO: Should this be std::numeric_limits<double>::max instead?
    matrix = Matrix(num_nodes, std::vector<distance_t>(num_nodes, std::numeric_limits<distance_t>::infinity()));
    for (int u = 0; u < num_nodes; u++)
    {

        matrix[u][u] = 0;
    }
    for (int cnt = 0; cnt < num_edges; cnt++)
    {
        Node u, v;
        distance_t dist;
        input_file >> u >> v >> dist;

        matrix[u][v] = dist;
    }

    input_file.close();
}

void GraphReader::write_to_file(std::string file_name)
{
    std::ofstream output_file;
    output_file.open(file_name);
    output_file << num_nodes << std::endl;
    apsp();
    for (int row = 0; row < num_nodes; row++)
    {
        for (int col = 0; col < num_nodes; col++)
        {
            output_file << apsp_matrix[row][col] << " ";
        }
        output_file << std::endl;
    }
    Penalties p(num_nodes, _lambda);
    p[root_node] = 0;
    for (penalty_t _ : p)
    {
        output_file << _ << " ";
    }
    output_file << std::endl;
    output_file.close();
}

Matrix GraphReader::get_matrix()
{
    return matrix;
}

Penalties GraphReader::get_penalties()
{
    Penalties penalties(num_nodes, _lambda);
    penalties[root_node] = 0;
    return penalties;
}

void GraphReader::apsp()
{
    apsp_matrix = matrix;
    for (int k = 0; k < num_nodes; k++)
    {
        for (int i = 0; i < num_nodes; i++)
        {
            for (int j = 0; j < num_nodes; j++)
            {

                if (apsp_matrix[i][j] > apsp_matrix[i][k] + apsp_matrix[k][j])
                {
                    apsp_matrix[i][j] = apsp_matrix[i][k] + apsp_matrix[k][j];
                }
            }
        }
    }
}

int main()
{
    GraphReader gr1("tests/graph1.txt", 1);
    gr1.write_to_file("tests/test1.txt");
    GraphReader gr2("tests/graph2.txt", 2);
    gr2.write_to_file("tests/test2.txt");
    return 0;
}
