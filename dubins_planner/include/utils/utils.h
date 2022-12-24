#include <iostream>
#include <vector>
#include <algorithm>

struct point_2d
{
    int x;
    int y;
};

struct RectangleObs
{
    point_2d ver_1;
    point_2d ver_2;
    point_2d ver_3;
    point_2d ver_4;

    bool sorted = false;

    std::vector<point_2d> points;

    void construct_list();

    void print();

    void sort();

    void inflate();
};

struct TriangleObs
{
    point_2d ver_1;
    point_2d ver_2;
    point_2d ver_3;

    bool sorted = false;

    std::vector<point_2d> points;

    void construct_list();

    void print();

    void sort();

    void inflate();
};

class Obstacles
{
    private:
        std::vector<RectangleObs> rect_obs_list;
        std::vector<TriangleObs> tri_obs_list;
    
    public:
        
        int get_rectangle_obst_count();

        int get_triangle_obst_count();

        void add_rectangle_obs(RectangleObs obs);

        void add_triangle_obs(TriangleObs obs);

        void print();

        bool is_inside_obs(point_2d point);
};

float area(int x1, int y1, int x2, int y2, int x3, int y3);

extern Obstacles map_obstacles;

extern std::vector<std::vector<std::vector<int>>> obstacle_list;

void print_obstacle_list();

void init_obstacle_structure();

