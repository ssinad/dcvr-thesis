#include <iostream>
#include "iterPCA.h"
#include "PcaReader.h"

int main(int argc, char const *argv[])
{
    // The first graph
    // root and another node
    // two arcs with weights equal to 2
    // sweep penalty: 1, 2, 3

    // penalty_t penalties[matrix_size];
    // u -> v <=> a_{uv} = 1
    // Do we count the root or not? Let's count it for now
    // Do we need constants?
    // int num_nodes = 2;
    // int matrix_size = 2 * num_nodes;

    // Penalties penalties(matrix_size);
    // Matrix c(matrix_size, std::vector<distance_t>(matrix_size, 0));

    // v[0] is the root? Yes

    // for (Node cnt = 0; cnt < num_nodes; cnt++)
    // {
    //     v.insert(cnt);
    // }

    // for (int row = 0; row < matrix_size; ++row)
    // {
    //     for (int col = 0; col < matrix_size; col++)
    //     {
    //         c[row][col] = 0;
    //     }
    // }
    // c[0][1] = 2;
    // c[1][0] = 2;

    // Copy the c matrix
    // Is there a better way?
    // Matrix c_copy(matrix_size, std::vector<distance_t>(matrix_size, 0));

    // for (int row = 0; row < matrix_size; ++row)
    // {
    //     for (int col = 0; col < matrix_size; col++)
    //     {
    //         c_copy[row][col] = 0;
    //     }
    // }
    // c_copy[0][1] = 2;
    // c_copy[1][0] = 2;

    std::string test("tests/");
    if (argc == 1)
    {
        std::cout << "Using default test\n";
        test += "test1.txt";
    }
    else
    {
        test += argv[1];
    }
    std::cout << "Testing file: " << test << "\n";

    {
        PcaReader pr1(test);

        Node last_node = pr1.get_num_nodes() - 1;
        Vertices v = pr1.get_vertices();
        Penalties penalties = pr1.get_penalties();
        Matrix c_copy = pr1.get_matrix();
        penalty_t theta = 0;

        Arborescence a = iterPCA(v, c_copy, penalties, theta, last_node, root_node);

        Matrix c_copy2 = pr1.get_matrix(); // just for reporting
        distance_t edge_cost = 0.0;
        std::cout << "\nArcs Bought:\n";
        for (std::pair<Node, Node> kv : a)
        {
            if (kv.first != root_node)
            {
                // outputs in the actual direction u->v of the arc,
                // which is "opposite" of how it is stored in the arborescence
                std::cout << kv.second << ": " << kv.first << std::endl;
            }
            edge_cost += c_copy2[kv.second][kv.first];
        }
        std::cout << "\nTotal edge cost: " << edge_cost << "\n\n";

        Penalties penalties_copy = pr1.get_penalties(); // just for reporting
        penalty_t penalty_cost = 0.0;
        std::cout << "Discarded Nodes:";
        for (Node x : v)
        {
            if (a.find(x) == a.end())
            {
                std::cout << ' ' << x;
                penalty_cost += penalties_copy[x];
            }
        }
        std::cout << "\n\nTotal penalty paid: " << penalty_cost << "\n\n";
        std::cout << "Overall cost: " << edge_cost + penalty_cost << "\n";

        //Arborescence a = iterPCA(v, c_copy, penalties, last_node, root_node);

        // for (penalty_t penalty = 1; penalty <= 3; penalty++)
        // {
        //     std::cout << "Penalty is " << penalty << std::endl;
        //     Node last_node = pr1.get_num_nodes() - 1;
        //     Vertices v = pr1.get_vertices();
        //     Matrix c_copy = pr1.get_matrix();
        //
        //     // for (int cnt = 0; cnt < num_nodes; cnt++)
        //     // {
        //     //     penalties[cnt] = penalty;
        //     // }
        //     Penalties penalties = pr1.get_penalties(penalty);
        //     // for (penalty_t p : penalties){
        //     //     std::cout << "The penalty is " << p << std::endl;
        //     // }
        //
        //     penalties[root_node] = 0;
        //     Arborescence a = iterPCA(v, c_copy, penalties, last_node, root_node);
        //     for (std::pair<Node, Node> kv : a)
        //     {
        //         std::cout << kv.first << ": " << kv.second << std::endl;
        //     }
        //     std::cout << std::endl;
        // }
    }

    // {
    //     PcaReader pr2("tests/test2.txt");
    //
    //     for (penalty_t penalty = 1; penalty <= 6; penalty++)
    //     {
    //         Node last_node = pr2.get_num_nodes() - 1;
    //         Vertices v = pr2.get_vertices();
    //         Matrix c_copy = pr2.get_matrix();
    //         std::cout << "Penalty is " << penalty << std::endl;
    //         // for (int cnt = 0; cnt < num_nodes; cnt++)
    //         // {
    //         //     penalties[cnt] = penalty;
    //         // }
    //         Penalties penalties = pr2.get_penalties(penalty);
    //         // Penalties penalties = pr2.get_penalties();
    //         // for (penalty_t p : penalties){
    //         //     std::cout << "The penalty is " << p << std::endl;
    //         // }
    //
    //         penalties[root_node] = 0;
    //         Arborescence a = iterPCA(v, c_copy, penalties, last_node, root_node);
    //         for (std::pair<Node, Node> kv : a)
    //         {
    //             std::cout << kv.first << ": " << kv.second << std::endl;
    //         }
    //         std::cout << std::endl;
    //     }
    // }

    /* code */
    return 0;
}