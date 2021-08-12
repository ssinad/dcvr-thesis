#include <iostream>
#include <unordered_map>
#include <dirent.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <thread>
#include "DatasetReader.h"
#include "cycle_dcvr.hpp"
#include "Path.h"

using namespace std::chrono;

std::vector<std::string> read_all_datasets_in_directory(std::string directory)
{
    std::vector<std::string> filenames;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(directory.c_str())) != NULL)
    {
        
        while ((ent = readdir(dir)) != NULL)
        {
            
            std::string tmp(ent->d_name);
            int l = tmp.length();
            if (l < 4)
                continue;
            
            if (tmp.substr(l - 4, 4) == ".txt")
            {
                filenames.push_back(tmp);
            }
        }
        closedir(dir);
    }
    return filenames;
}

void map_function(distance_t distance, std::string filename="", std::string filename_base="", bool redirect_output=true)
{
    DatasetReader dr;
    if (redirect_output)
    {
        std::string log_filename = filename_base + "/" + filename + ".log";
        freopen("error.log", "w", stderr);
        freopen(log_filename.c_str(), "w", stdout);
    }
    
    std::string actual_filename = (filename_base != "")?(filename_base + "/" + filename):filename;
    if (filename != ""){
        std::cout << filename << std::endl;
    }
    dr.read_file(actual_filename);
    std::unordered_map<int, Path> path_map;
    std::unordered_map<int, double> values;
    std::list<Path> paths;
    int n = dr.get_vertices().size();
    std::cout << "Number of nodes: " << n << std::endl;

    auto start = clock();
    paths = cycle_dcvr(dr.get_vertices(), dr.get_root_node(), dr.get_matrix(), distance);
    auto stop = clock();
    auto duration = 1000.0 * (stop - start) / CLOCKS_PER_SEC;

    std::string paths_filename = actual_filename + ".paths";
    std::ofstream paths_file(paths_filename);
    paths_file << "[" << std::endl;
    bool first_path = true;
    for (Path p: paths)
    {
        bool first_node = true;
        if (!first_path)
        {
            paths_file << "," << std::endl;
        }
        paths_file << "[";
        for (Node n: p)
        {
            std::cout << n << " ";
            if (!first_node)
            {
                paths_file << ", " << n;
            }
            else
            {
                paths_file << n ;
            }
            first_node = false;
        }
        paths_file << "]";
        first_path = false;
        std::cout << std::endl;
    }
    paths_file << std::endl <<  "]";
    paths_file.flush();
    paths_file.close();
    std::cout << std::endl
                << "Execution time: " << duration << " ms";
}

int main(int argc, char* argv[])
{
    
    // DatasetReader dr;
    
    if (argc != 1){
        std::string filename = ""; //, filename_base="";
        distance_t distance_limit;
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
        map_function(distance_limit, filename, "", false);
        return EXIT_SUCCESS;
    }
    Path p;
    int c = 10;
    std::string euclidean_base = "datasets/euclidean";
    std::vector<std::string> euclidean_filenames = read_all_datasets_in_directory(euclidean_base);
    
    for (std::string filename : euclidean_filenames)
    {
        int n;
        std::ifstream euclidean_file(euclidean_base + "/" + filename);
        euclidean_file >> n;
        euclidean_file.close();
        distance_t distance_limit = std::max(0.7 * sqrt(n) / c, sqrt(2));
        map_function(distance_limit, filename, euclidean_base);
    }
    
    std::string tsp_base = "datasets/tsp";
    std::list <std::string> tsp_filenames = {"pr76", "gr96", "gr202"};
    for (std::string filename : tsp_filenames)
    {
        distance_t distance_limit;
        std::ifstream distance_limit_file(tsp_base + "/" + filename + ".opt.tour.len");
        distance_limit_file >> distance_limit;
        distance_limit_file.close();
        map_function(distance_limit, filename + ".txt", tsp_base);
    }

    std::vector<std::string> filenames{
        "datasets/small-sample.txt",
        "datasets/christofides-et-al-1979-set-m/M-n101-k10.txt",
        "datasets/christofides-et-al-1979-set-m/M-n151-k12.txt",
        "datasets/christofides-et-al-1979-set-m/M-n121-k07.txt",
    };
    std::vector<distance_t> distances{
        4,
        100,
        100,
        100,
    };
    int cnt = 0;
    for (std::string filename : filenames)
    {
        map_function(distances[cnt], filename);
        cnt++;
    }
    return EXIT_SUCCESS;
}
