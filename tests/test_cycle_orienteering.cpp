#include "DatasetReader.h"
#include "orienteering.h"
#include <iostream>
#include <cassert>

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
        std::unordered_map<Node, OrienteeringInfo> best_path_info_map, best_bound_info_map;
        std::cout << "{" << std::endl;
        std::cout << "\"number of nodes\": " << dr.get_vertices().size() << " , " << std::endl;
        std::cout << "\"distance bound\": " << distance_limit << " , " << std::endl;
        auto p = cycle_orienteering(dr.get_vertices(), dr.get_root_node(), dr.get_matrix(), dr.get_penalties(), distance_limit, best_path_info_map, best_bound_info_map);
        std::cout << "\"path nodes\": [ ";
        for (Node n: p.second)
        {
            std::cout << n << ", ";
        }
        std::cout << "] ," << std::endl;
        // std::clog <<"Upper bounds: ";
        penalty_t bound = 0;
        OrienteeringInfo best_bound_info, best_path_info;
        for (auto tmp: best_bound_info_map){
            if (bound < tmp.second.upper_bound){
                bound = tmp.second.upper_bound;
                best_bound_info = tmp.second;
            }
            std::clog << tmp.first << ": " << tmp.second.upper_bound << " " << tmp.second.lambda << std::endl;
        }
        penalty_t reward = 0;
        for (auto tmp: best_path_info_map){
            if (reward < tmp.second.path_reward){
                reward = tmp.second.path_reward;
                best_path_info = tmp.second;
            }
            // std::clog << tmp.first << ": " << tmp.second.upper_bound << " ";
        }
        assert(best_bound_info.upper_bound <= 4 * best_path_info.path_reward);
        penalty_t path_reward = get_path_reward(p.second, dr.get_penalties());
        std::clog << best_bound_info.lambda << " " << best_bound_info.theta << std::endl;
        std::cout << "\"path reward\": " << path_reward << " , " << std::endl;
        std::cout << "\"tree reward\": " << best_path_info.arb_reward << " ," << std::endl;
        std::cout << "\"reward residue\": " << path_reward - 1 << " ," << std::endl;
        std::cout << "\"path distance\": " << get_path_distance(p.second, dr.get_matrix()) << " , " << std::endl;
        std::cout << "\"tree distance\": " << best_path_info.arb_distance << " ," << std::endl;
        std::cout << "\"upper bound\": " << best_bound_info.upper_bound << " ," << std::endl;
        std::cout << "} ," << std::endl;
        return EXIT_SUCCESS;
    }
    return 0;
}