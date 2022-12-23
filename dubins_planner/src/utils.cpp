#include "utils/utils.h"

int atest = 6;

void print_obstacle_list
()
{
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
}

// void inflate_rectangle
// (std::vector<std::vector<int>> &inp)
// {
//     int biggest_col = 0;
//     int biggest_row = 0;

//     for 
//     (size_t i = 0; i < inp.size(); i++)
//     {
//         if
//         (inp[i][0] > biggest_row)
//         {
//             biggest_row = inp[i][0];
//         }

//         if
//         (inp[i][1] > biggest_col)
//         {
//             biggest_col = inp[i][1];
//         }
//     }
    
//     for 
//     (size_t i = 0; i < inp.size(); i++)
//     {
//         if
//         (inp[i][0] < biggest_row)
//         {
//             inp[i][0] = inp[i][0] - 1;
//         }
//         else
//         {
//             inp[i][0] = inp[i][0] + 1;
//         }

//         if
//         (inp[i][1] <  biggest_col)
//         {
//             inp[i][1] = inp[i][1] - 1;
//         }
//         else
//         {
//             inp[i][1] = inp[i][1] + 1;
//         }
//     }
// }

void inflate_obstacles
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


void inflate_obstacles_list
(std::vector<std::vector<std::vector<int>>> &inp)
{
    for 
    (size_t i = 0; i < inp.size(); i++)
    {
        inflate_obstacles(inp[i]);
    }
}