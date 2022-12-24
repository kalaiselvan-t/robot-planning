#include "utils/utils.h"

Obstacles map_obstacles;

/*=======================================
=========Obstacles Class Methods=========
=========================================*/

// RectangleObs

void RectangleObs::construct_list
()
{
    points = {ver_1,ver_2,ver_3};
}

void RectangleObs::print
()
{
    std::cout << "==========Rectangle Obstacle==========\n";
    for 
    (size_t i = 0; i < points.size(); i++)
    {
        std::cout << points[i].x << ", " << points[i].y << std::endl;
    }
}

void RectangleObs::sort
()
{ 
    std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.y < rhs.y;});
    std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.x < rhs.x;});
    sorted = true;
}

void RectangleObs::inflate
()
{
    int big_row = 0;
    int big_col = 0;
    int small_row = 0;
    int small_col = 0;

    for 
    (size_t i = 0; i < points.size(); i++)
    {
        if
        (big_row == 0 && small_row == 0){

            big_row = points[i].x;
            small_row = points[i].x;
            big_col =points[i].y;
            small_col = points[i].y;
        }
        else
        {
            if
            (points[i].x < small_row)
            {
                small_row = points[i].x;
            }

            if
            (points[i].x > big_row)
            {
                big_row = points[i].x;
            }

            if
            (points[i].y < small_col)
            {
                small_col = points[i].y;
            }

            if
            (points[i].y > big_col)
            {
                big_col = points[i].y;
            }
        }
    }
    
    for 
    (size_t i = 0; i < points.size(); i++)
    {

        if
        (points[i].x == small_row)
        {
            points[i].x = points[i].x - 1;
        }
        
        if
        (points[i].x == big_row)
        {
            points[i].x = points[i].x + 1;
        }

        if
        (points[i].y == small_col)
        {
            points[i].y = points[i].y - 1;
        }
        
        if
        (points[i].y == big_col)
        {
            points[i].y = points[i].y + 1;
        }
    }
}

// TriangleObs

void TriangleObs::construct_list
()
{
    points = {ver_1,ver_2,ver_3};
}

void TriangleObs::print
()
{   
    std::cout << "==========Triangle Obstacle==========\n";
    for 
    (size_t i = 0; i < points.size(); i++)
    {
        std::cout << points[i].x << ", " << points[i].y << std::endl;
    }
}

void TriangleObs::sort
()
{ 
    std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.y < rhs.y;});
    std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.x < rhs.x;});
    sorted = true;
}

void TriangleObs::inflate()
{
    int big_row = 0;
    int big_col = 0;
    int small_row = 0;
    int small_col = 0;

    for 
    (size_t i = 0; i < points.size(); i++)
    {
        if
        (big_row == 0 && small_row == 0){

            big_row = points[i].x;
            small_row = points[i].x;
            big_col =points[i].y;
            small_col = points[i].y;
        }
        else
        {
            if
            (points[i].x < small_row)
            {
                small_row = points[i].x;
            }

            if
            (points[i].x > big_row)
            {
                big_row = points[i].x;
            }

            if
            (points[i].y < small_col)
            {
                small_col = points[i].y;
            }

            if
            (points[i].y > big_col)
            {
                big_col = points[i].y;
            }
        }
    }
    
    for 
    (size_t i = 0; i < points.size(); i++)
    {

        if
        (points[i].x == small_row)
        {
            points[i].x = points[i].x - 1;
        }
        
        if
        (points[i].x == big_row)
        {
            points[i].x = points[i].x + 1;
        }

        if
        (points[i].y == small_col)
        {
            points[i].y = points[i].y - 1;
        }
        
        if
        (points[i].y == big_col)
        {
            points[i].y = points[i].y + 1;
        }
    }
}
/*----------------------------------------*/

/*=======================================
=========Obstacles Class Methods=========
=========================================*/

int Obstacles::get_rectangle_obst_count
()
{
    return rect_obs_count;
}

int Obstacles::get_triangle_obst_count
()
{
    return tri_obs_count;
}

void Obstacles::add_rectangle_obs
(RectangleObs obs)
{
    obs.inflate();
    rect_obs_list.push_back(obs);
    rect_obs_count = rect_obs_list.size();
}

void Obstacles::add_triangle_obs
(TriangleObs obs)
{   
    obs.inflate();
    tri_obs_list.push_back(obs);
    tri_obs_count = tri_obs_list.size();
}

void Obstacles::print
()
{
    std::cout << "Rectangle obstacle list: \n";
    for 
    (size_t i = 0; i < rect_obs_list.size(); i++)
    {
        rect_obs_list[i].print();
    }

    std::cout << "Triangle obstacle list: \n";
    for 
    (size_t i = 0; i < tri_obs_list.size(); i++)
    {
        tri_obs_list[i].print();
    }
}

int Obstacles::is_inside_obs
(point_2d point)
{
    
}
/*------------------------------------------------*/

// Prints the current values in obstacle_list
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

// Populates the obstacles class from given obstacles list
void init_obstacle_structure
()
{
    for 
    (size_t i = 0; i < obstacle_list.size(); i++)
    {
        if 
        (obstacle_list[i].size() == 4)
        {   
            RectangleObs obs;     
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
            // std::cout << "======================\n";
            // obs.print();
            // obs.sort();
            map_obstacles.add_rectangle_obs(obs);
            // std::cout << "-------------------------\n";
            // obs.print();
            // std::cout << "area: " << obs.area() << std::endl;
            // std::cout << "======================\n";
        }

        if 
        (obstacle_list[i].size() == 3)
        {   
            TriangleObs obs;     
            // std::cout << "iteration: " << i << std::endl;
            obs.ver_1.x = obstacle_list[i][0][0];
            obs.ver_1.y = obstacle_list[i][0][1];
            obs.ver_2.x = obstacle_list[i][1][0];
            obs.ver_2.y = obstacle_list[i][1][1];
            obs.ver_3.x = obstacle_list[i][2][0];
            obs.ver_3.y = obstacle_list[i][2][1];
            obs.construct_list();
            // std::cout << "======================\n";
            // obs.print();
            // obs.sort();
            map_obstacles.add_triangle_obs(obs);
            // std::cout << "--------------------------\n";
            // obs.print();
            // std::cout << "======================\n";
            // std::cout << "area: " << obs.area() << std::endl;
        }          
    }   
}