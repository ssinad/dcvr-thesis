#include "DatasetReader.h"
#include "orienteering.h"
#include <iostream>

int main()
{
    DatasetReader dr;
    // Path p;
    std::vector<std::string> filenames{
        "datasets/christofides-et-al-1979-set-m/M-n101-k10.txt",
        "datasets/christofides-et-al-1979-set-m/M-n151-k12.txt",
        "datasets/christofides-et-al-1979-set-m/M-n121-k07.txt"};
    for (std::string filename : filenames)
    {
        dr.read_file(filename, true);
        std::unordered_map<Node, penalty_t> upper_bounds;
        auto p = orienteering(dr.get_vertices(), dr.get_root_node(), dr.get_matrix(), dr.get_penalties(), 100, upper_bounds);
        for (Node n : p.second)
        {
            std::cout << n << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}