#include "DatasetReader.h"
#include "orienteering.h"
#include <iostream>

int main(int argc, char ** argv)
{
    DatasetReader dr;
    std::string filename = "";
    distance_t distance_limit = 100;
    if (argc != 1)
    {
        for (int ind = 1; ind < argc; ++ind){
            std::string current_arg = argv[ind];
            if (current_arg == "-f"){
                filename = std::string(argv[ind + 1]);
            }
            if (current_arg == "-D"){
                double tmp = std::stod(std::string(argv[ind + 1]));
                distance_limit = tmp;
            }
        }
        dr.read_file(filename, true);
        std::unordered_map<Node, penalty_t> upper_bounds;
        auto p = orienteering(dr.get_vertices(), dr.get_root_node(), dr.get_matrix(), dr.get_penalties(), distance_limit, upper_bounds);
        for (Node n: p.second)
        {
            std::cout << n << " ";
        }
        std::cout << std::endl;
        return EXIT_SUCCESS;
    }
    
    
    
    std::vector<std::string> filenames{
        "datasets/christofides-et-al-1979-set-m/M-n101-k10.txt",
        "datasets/christofides-et-al-1979-set-m/M-n151-k12.txt",
        "datasets/christofides-et-al-1979-set-m/M-n121-k07.txt"};
    for (filename : filenames)
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