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
        dr.read_file(filename, Dataset_type::ORIENTEERING);
        std::unordered_map<Node, BestPathInfo> best_path_info_map;
        std::unordered_map<Node, BoundInfo> best_bound_info_map;
        std::cout << "Number of nodes: " << dr.get_vertices().size() << std::endl;
        auto p = rooted_orienteering(dr.get_vertices(), dr.get_root_node(), dr.get_matrix(), dr.get_penalties(), distance_limit, best_path_info_map, best_bound_info_map);
        std::cout << "path nodes: [ ";
        for (Node n: p.second)
        {
            std::cout << n << " , ";
        }
        std::cout << "]" << std::endl;
        penalty_t bound = 0;
        for (auto tmp: best_bound_info_map){
            if (bound < tmp.second.upper_bound){
                bound = tmp.second.upper_bound;
            }
        }
        penalty_t path_reward = get_path_reward(p.second, dr.get_penalties());
        std::cout << "Path reward: " << path_reward << std::endl;
        std::cout << "Path distance: " << get_path_distance(p.second, dr.get_matrix()) << std::endl;
        std::cout << "Upper bound: " << bound << std::endl;
        std::cout << "Reward residue: " << path_reward - 1 << std::endl;
        return EXIT_SUCCESS;
    }
    return 0;
}