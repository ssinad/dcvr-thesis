#include "rooted_orienteering.h"
#include "Path.h"
#include "Matrix.h"
#include <iostream>

using namespace std;

int main()
{
    Path p = {0, 1, 2, 3, 4};
    // distance_t distance_limit_D = 5;
    Rewards rewards = {0, 1, 2, 3, 4};
    int matrix_size = 5;
    Matrix distances(matrix_size, std::vector<distance_t>(matrix_size, 0));
    distances[0][1] = 1;
    distances[1][2] = 1;
    distances[2][3] = 1;
    distances[3][4] = 1;
    distances[0][2] = 1.414;
    distances[0][3] = 2.236;
    distances[0][4] = 3.162;
    for (distance_t distance_limit_D = 3.5; distance_limit_D <= 5; distance_limit_D+= 0.5){
        Path best_path = cut_path(p, distances, rewards, 0, distance_limit_D);
        for (Node _: best_path){
            cout << _ << " ";
        }
        cout << endl;
    }

    return EXIT_SUCCESS;
}