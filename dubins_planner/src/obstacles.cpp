#include <iostream>
#include "utils/utils.h"


using namespace std;

int main()
{
    // vector<vector<int>> test_obs = {{6,3},{11,3},{11,4},{6,4}};
    // print_obstacle_list();
    // inflate_obstacles_list(obstacle_list);
    // print_obstacle_list();
    // cout << "========================\n";
    init_obstacle_structure();

    map_obstacles.print();
    // cout << "rectangle count: "<< map_obstacles.get_rectangle_obst_count() << endl;
    // cout << "triangle count: "<< map_obstacles.get_triangle_obst_count() << endl;
}