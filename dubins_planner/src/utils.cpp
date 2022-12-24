#include "utils/utils.h"

Obstacles map_obstacles;

/*=======================================
=========Obstacles Class Methods=========
=========================================*/

// RectangleObs

void RectangleObs::construct_list
()
{
    points = {ver_1, ver_2, ver_3, ver_4};
}

void RectangleObs::print
()
{
    std::cout << "==========Rectangle Obstacle==========\n";
    std::cout << "size: " << points.size() << std::endl;
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
    std::cout << "size: " << points.size() << std::endl;
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
    return rect_obs_list.size();
}

int Obstacles::get_triangle_obst_count
()
{
    return tri_obs_list.size();
}

void Obstacles::add_rectangle_obs
(RectangleObs obs)
{
    obs.inflate();
    rect_obs_list.push_back(obs);
}

void Obstacles::add_triangle_obs
(TriangleObs obs)
{   
    obs.inflate();
    tri_obs_list.push_back(obs);
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

bool Obstacles::is_inside_obs
(point_2d point)
{
    int count = -1;
    for 
    (size_t i = 0; i < rect_obs_list.size(); i++)
    {
        // std::cout << "Rectangle iter: " << i << std::endl;

        float Area = area(rect_obs_list[i].points[0].x, rect_obs_list[i].points[0].y, rect_obs_list[i].points[1].x, rect_obs_list[i].points[1].y, rect_obs_list[i].points[2].x, rect_obs_list[i].points[2].y) +
                    
                    area(rect_obs_list[i].points[0].x, rect_obs_list[i].points[0].y, rect_obs_list[i].points[3].x, rect_obs_list[i].points[3].y, rect_obs_list[i].points[2].x, rect_obs_list[i].points[2].y);
        
        float A1 = area(point.x, point.y, rect_obs_list[i].points[0].x, rect_obs_list[i].points[0].y, rect_obs_list[i].points[1].x, rect_obs_list[i].points[1].y);

        float A2 = area(point.x, point.y, rect_obs_list[i].points[1].x, rect_obs_list[i].points[1].y, rect_obs_list[i].points[2].x, rect_obs_list[i].points[2].y);

        float A3 = area(point.x, point.y, rect_obs_list[i].points[2].x, rect_obs_list[i].points[2].y, rect_obs_list[i].points[3].x, rect_obs_list[i].points[3].y);

        float A4 = area(point.x, point.y, rect_obs_list[i].points[0].x, rect_obs_list[i].points[0].y, rect_obs_list[i].points[3].x, rect_obs_list[i].points[3].y);

        if 
        (Area == A1 + A2 + A3 + A4)
        {
            count += 1;
            // std::cout << "in here\n";
        }
    }
    
    for 
    (size_t i = 0; i < tri_obs_list.size(); i++)
    {
        // std::cout << "Triangle iter: " << i << std::endl;

        float Area = area(tri_obs_list[i].points[0].x, tri_obs_list[i].points[0].y, tri_obs_list[i].points[1].x, tri_obs_list[i].points[1].y, tri_obs_list[i].points[2].x, tri_obs_list[i].points[2].y);

        float A1 = area(point.x, point.y, tri_obs_list[i].points[1].x, tri_obs_list[i].points[1].y, tri_obs_list[i].points[2].x, tri_obs_list[i].points[2].y);

        float A2 = area(tri_obs_list[i].points[0].x, tri_obs_list[i].points[0].y, point.x, point.y, tri_obs_list[i].points[2].x, tri_obs_list[i].points[2].y);

        float A3 = area(tri_obs_list[i].points[0].x, tri_obs_list[i].points[0].y, tri_obs_list[i].points[1].x, tri_obs_list[i].points[1].y, point.x, point.y);

        if 
        (Area == A1 + A2 + A3)
        {
            count += 1;
            // std::cout << "in here\n";
        }
    }

    if
    (count < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
/*------------------------------------------------*/

float area(int x1, int y1, int x2, int y2, int x3, int y3)
{
    return std::abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
}

// Prints the current values in obstacle_list
void print_obstacle_list
()
{
    for 
    (size_t i = 0; i < obstacle_list.size(); i++)
    {
        std::cout << "obstacle: " << i << std::endl;
        std::cout << "size: " << obstacle_list[i].size() << std::endl;
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
            
            obs.ver_1.x = obstacle_list[i][0][0];
            obs.ver_1.y = obstacle_list[i][0][1];
            obs.ver_2.x = obstacle_list[i][1][0];
            obs.ver_2.y = obstacle_list[i][1][1];
            obs.ver_3.x = obstacle_list[i][2][0];
            obs.ver_3.y = obstacle_list[i][2][1];
            obs.ver_4.x = obstacle_list[i][3][0];
            obs.ver_4.y = obstacle_list[i][3][1];

            obs.construct_list();
            map_obstacles.add_rectangle_obs(obs);
        }

        if 
        (obstacle_list[i].size() == 3)
        {   
            TriangleObs obs;     
    
            obs.ver_1.x = obstacle_list[i][0][0];
            obs.ver_1.y = obstacle_list[i][0][1];
            obs.ver_2.x = obstacle_list[i][1][0];
            obs.ver_2.y = obstacle_list[i][1][1];
            obs.ver_3.x = obstacle_list[i][2][0];
            obs.ver_3.y = obstacle_list[i][2][1];
            obs.construct_list();
            
            map_obstacles.add_triangle_obs(obs);
        }          
    }   
}