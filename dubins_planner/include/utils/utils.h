#include <iostream>
#include <vector>
#include <algorithm>
// #include <boost/fusion/adapted/struct.hpp>
// #include <boost/fusion/include/for_each.hpp>
// #include <boost/phoenix/phoenix.hpp>

// using boost::phoenix::arg_names::arg1;

void greet();

struct point_2d
{
    int x;
    int y;
};

struct rectangle_obstacle
{
    point_2d ver_1;
    point_2d ver_2;
    point_2d ver_3;
    point_2d ver_4;

    bool sorted = false;

    std::vector<point_2d> points;

    void construct_list
    ()
    {
        points = {ver_1,ver_2,ver_3,ver_4};
    }

    float area
    ()
    {
        return std::abs((ver_1.x * (ver_2.y - ver_3.y) + ver_2.x * (ver_3.y - ver_1.y) + ver_3.x * (ver_1.y - ver_2.y)) / 2.0);
    }

    void print
    ()
    {
        for 
        (size_t i = 0; i < points.size(); i++)
        {
            std::cout << points[i].x << ", " << points[i].y << std::endl;
        }
    }

    void sort
    ()
    { 
        std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.y < rhs.y;});
        std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.x < rhs.x;});
        sorted = true;
    }

};

struct triangle_obstacle
{
    point_2d ver_1;
    point_2d ver_2;
    point_2d ver_3;

    bool sorted = false;

    std::vector<point_2d> points;

    void construct_list
    ()
    {
        points = {ver_1,ver_2,ver_3};
    }

    void print
    ()
    {
        for 
        (size_t i = 0; i < points.size(); i++)
        {
            std::cout << points[i].x << ", " << points[i].y << std::endl;
        }
    }

    void sort
    ()
    { 
        std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.y < rhs.y;});
        std::stable_sort(std::begin(points), std::end(points), [] (const auto& lhs, const auto& rhs) {return lhs.x < rhs.x;});
        sorted = true;
    }

};

class obstacles
{
    private:
        std::vector<rectangle_obstacle> rect_obs_list;
        std::vector<triangle_obstacle> tri_obs_list;
        int rect_obs_count = rect_obs_list.size();
        int tri_obs_count = rect_obs_list.size();
    
    public:
        
        int get_rectangle_obst_count
        ()
        {
            return rect_obs_count;
        }

        int get_triangle_obst_count
        ()
        {
            return tri_obs_count;
        }

        void add_rectangle_obs
        (rectangle_obstacle obs)
        {
            // inflate_obstacle(obs);
            rect_obs_list.push_back(obs);
            rect_obs_count = rect_obs_list.size();
        }

        void add_triangle_obs
        (triangle_obstacle obs)
        {
            tri_obs_list.push_back(obs);
            tri_obs_count = tri_obs_list.size();
        }
};

extern obstacles map_obstacles;

extern std::vector<std::vector<std::vector<int>>> obstacle_list;

void print_obstacle_list();

void inflate_obstacle(std::vector<std::vector<int>> &);

void inflate_obstacles_list(std::vector<std::vector<std::vector<int>>> &);

void init_obstacle_structure();

