#include <iostream>
#include "utils/utils.h"


using namespace std;

int main()
{
    init_obstacle_structure();

    point_2d p = {7,16};

    if
    (map_obstacles.is_inside_obs(p))
    {
        std::cout << "Inside" << std::endl;
    }
    else
    {
        std::cout << "Outside" << std::endl;
    }

    return 0;
}