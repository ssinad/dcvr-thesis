#include "orienteering.h"
#include <iostream>

using namespace std;

int main()
{
    Node r = 0, t = 5;
    Arborescence a = {
        {1, r},
        {2, 1},
        {3, 2},
        {4, 3},
        {t, 4},
        {6, r},
        {7, 6},
        {8, 1},
        {9, 1},
        {10, 2},
        {11, 2},
        {12, 3},
        {13, 3},
        {14, 4},
        {15, 4},
        {16, 5},
        {17, 5},
        {18, 17}};
    
        Path s_t_path = get_path(a, r, t, false);
        for (Node n : s_t_path)
        {
            std::cout << n << " ";
        }
        std::cout << std::endl;
    
        Path s_t_path_triangle = get_path(a, r, t, true);
        for (Node n : s_t_path_triangle)
        {
            std::cout << n << " ";
        }
        std::cout << std::endl;
    
    return 0;
}
