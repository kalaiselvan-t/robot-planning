#include <iostream>
#include "utils/utils.h"


using namespace std;

int main()
{
    print_obstacle_list();
    inflate_obstacles_list(obstacle_list);
    print_obstacle_list();
}