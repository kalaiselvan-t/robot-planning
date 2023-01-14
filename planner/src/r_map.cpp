#include "../include/planner/r_map.h"

// Data Struct Initialisations
int resolution = 100;
float resol = 0.5;
int max_border_x = 20;
int max_border_y = 20;

// random_device rd;
// mt19937 gen(rd());
// uniform_real_distribution<float> dis_x(0,max_border_x);
// uniform_real_distribution<float> dis_y(0, max_border_y);

boost_point pt;
Nodes roadmap_nodes;
Grid map_grid;
vector<ObstacleTypes> obs_list;
vector<pair<float, float>> obs_pts_list;

// Function Declarations

// Point

Point::Point(ros_point inp)
{
    ros_pt = inp;
    get_boost_pt();
}

Point::Point(boost_point inp)
{
    boost_pt = inp;
    get_ros_pt();
}

Point::Point()
{
}

void Point::get_boost_pt()
{
    boost_pt.x(ros_pt.x);
    boost_pt.y(ros_pt.y);
}

void Point::get_ros_pt()
{
    ros_pt.x = boost_pt.x();
    ros_pt.y = boost_pt.y();
}

// Grid
void Grid::create_grid()
{
    for (float i = -max_border_x / 2; i <= max_border_x / 2; i++)
    {
        vector<boost_point> temp;
        for (float j = -max_border_y / 2; j <= max_border_y / 2; j++)
        {
            pt.x(static_cast<float>(i));
            pt.y(static_cast<float>(j));
            // grid[i][j] = pt;
            temp.push_back(pt);
        }
        grid.push_back(temp);
    }
}

void Grid::print_grid()
{
    for (size_t i = 0; i < grid.size(); i++)
    {
        for (size_t j = 0; j < grid[i].size(); j++)
        {
            cout << "x: " << grid[i][j].x() << ", y: " << grid[i][j].y() << endl;
        }
        cout << "-----------------------\n";
    }
}

// ObstacleTypes

ros_polygon ObstacleTypes::boost2ros(boost_polygon inp)
{
    ros_polygon ret;
    for (const auto &p : inp.outer())
    {

        ros_point temp;
        temp.x = p.x();
        temp.y = p.y();
        ret.points.push_back(temp);
    }
    return ret;
}

boost_polygon ObstacleTypes::ros2boost(ros_polygon inp)
{
    boost_polygon ret;

    for (size_t i = 0; i < inp.points.size(); i++)
    {
        boost_point temp;
        temp.x(inp.points[i].x);
        temp.y(inp.points[i].y);
        ret.outer().push_back(temp);
    }
    return ret;
}

void ObstacleTypes::get_boost_poly()
{
    boost_poly = ros2boost(ros_poly);
}

void ObstacleTypes::get_ros_poly()
{
    ros_poly = boost2ros(boost_poly);
}

bool operator==(const boost_point &lhs, const boost_point &rhs)
{
    return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

// Util fns

float round_up2(float inp, int places)
{
    int no = pow(10, places);

    return floorf(inp * no) / no;
}

// Node create_random_node
//     ()
// {
//     Node node;
//     node.x = round_up(dis_x(gen),2);
//     node.y = round_up(dis_y(gen),2);

//     return node;
// }

void test_link()
{
    cout << "Connected\n";
}