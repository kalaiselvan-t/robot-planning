#include "utils/utils.h"
// #include <algorithm>

obstacles map_obstacles;

void greet
()
{
    std::cout << "Greetings\n";
}

bool yComparator
(const point_2d & lhs, const point_2d & rhs)
{
    return lhs.y < rhs.y;
}

bool xComparator
(const point_2d & lhs, const point_2d & rhs)
{
    return lhs.x < rhs.x;
}

// Prints the current values in obstacle_list
void print_obstacle_list
()
{
    // if 
    // (sort)
    // {
    //     for 
    //     (size_t i = 0; i < obstacle_list.size(); i++)
    //     {
    //         std::stable_sort(obstacle_list[i].begin(), obstacle_list[i].end(), yComparator());
    //         std::stable_sort(obstacle_list[i].begin(), obstacle_list[i].end(), xComparator());

    //     }
        
    // }
    // else
    // {
        for 
        (size_t i = 0; i < obstacle_list.size(); i++)
        {
            std::cout << "obstacle: " << i << std::endl;
            for 
            (size_t j = 0; j < obstacle_list[i].size(); j++)
            {
                std::cout << obstacle_list[i][j][0] << ", " << obstacle_list[i][j][1] << std::endl;
            }
        }
//     }
    
}

//inflates given obstacle by 1 unit
void inflate_obstacle
(std::vector<std::vector<int>> &inp)
{
    int big_row = 0;
    int big_col = 0;
    int small_row = 0;
    int small_col = 0;

    for 
    (size_t i = 0; i < inp.size(); i++)
    {
        if
        (big_row == 0 && small_row == 0){

            big_row = inp[i][0];
            small_row = inp[i][0];
            big_col =inp[i][1];
            small_col = inp[i][1];
        }
        else
        {
            if
            (inp[i][0] < small_row)
            {
                small_row = inp[i][0];
            }

            if
            (inp[i][0] > big_row)
            {
                big_row = inp[i][0];
            }

            if
            (inp[i][1] < small_col)
            {
                small_col = inp[i][1];
            }

            if
            (inp[i][1] > big_col)
            {
                big_col = inp[i][1];
            }
        }
    }
    
    for 
    (size_t i = 0; i < inp.size(); i++)
    {

        if
        (inp[i][0] == small_row)
        {
            inp[i][0] = inp[i][0] - 1;
        }
        
        if
        (inp[i][0] == big_row)
        {
            inp[i][0] = inp[i][0] + 1;
        }

        if
        (inp[i][1] == small_col)
        {
            inp[i][1] = inp[i][1] - 1;
        }
        
        if
        (inp[i][1] == big_col)
        {
            inp[i][1] = inp[i][1] + 1;
        }
    }
}

// Loops through the obstacle list and inflates each obstacle
void inflate_obstacles_list
(std::vector<std::vector<std::vector<int>>> &inp)
{
    for 
    (size_t i = 0; i < inp.size(); i++)
    {
        inflate_obstacle(inp[i]);
    }
}

void init_obstacle_structure
()
{
    // int count = -1;
    for 
    (size_t i = 0; i < obstacle_list.size(); i++)
    {
        if 
        (obstacle_list[i].size() == 4)
        {   
            rectangle_obstacle obs;     
            // std::cout << "iteration: " << i << std::endl;
            obs.ver_1.x = obstacle_list[i][0][0];
            obs.ver_1.y = obstacle_list[i][0][1];
            obs.ver_2.x = obstacle_list[i][1][0];
            obs.ver_2.y = obstacle_list[i][1][1];
            obs.ver_3.x = obstacle_list[i][2][0];
            obs.ver_3.y = obstacle_list[i][2][1];
            obs.ver_4.x = obstacle_list[i][3][0];
            obs.ver_4.y = obstacle_list[i][3][1];

            obs.construct_list();
            obs.print();
            obs.sort();
            map_obstacles.add_rectangle_obs(obs);
            std::cout << "======================\n";
            obs.print();
            std::cout << "area: " << obs.area() << std::endl;
        }

        if 
        (obstacle_list[i].size() == 3)
        {   
            triangle_obstacle obs;     
            // std::cout << "iteration: " << i << std::endl;
            obs.ver_1.x = obstacle_list[i][0][0];
            obs.ver_1.y = obstacle_list[i][0][1];
            obs.ver_2.x = obstacle_list[i][1][0];
            obs.ver_2.y = obstacle_list[i][1][1];
            obs.ver_3.x = obstacle_list[i][2][0];
            obs.ver_3.y = obstacle_list[i][2][1];
            obs.construct_list();
            obs.print();
            obs.sort();
            map_obstacles.add_triangle_obs(obs);
            std::cout << "======================\n";
            obs.print();
            // std::cout << "area: " << obs.area() << std::endl;
        }          
    }   
}