#include <iostream>
#include "iterPCA.hpp"
#include "PcaReader.h"

int main(int argc, char const *argv[])
{
    Node root_node = 0;
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
    }
    return 0;
}