#include "../include/planner/r_map.h"

// Data Struct Initialisations
int resolution = 100;
int max_border_x = 20;
int max_border_y = 20;

// random_device rd;
// mt19937 gen(rd());
// uniform_real_distribution<float> dis_x(0,max_border_x);
// uniform_real_distribution<float> dis_y(0, max_border_y);

point pt;
Nodes roadmap_nodes;
Grid map_grid;
vector<ObstacleTypes> obs_list;
vector<pair<float,float>> obs_pts_list;

// Function Declarations

//Point

Point::Point(geometry_msgs::msg::Point32 inp)
{
    ros_pt = inp;
    get_boost_pt();
}

Point::Point(point inp)
{
    boost_pt = inp;
    get_ros_pt();
}

Point::Point()
{

}

void Point::get_boost_pt
()
{
    boost_pt.x(ros_pt.x);
    boost_pt.y(ros_pt.y);
}

void Point::get_ros_pt
()
{
    ros_pt.x = boost_pt.x();
    ros_pt.y = boost_pt.y();
}

//Grid
void Grid::create_grid
()
{
    for 
        (int i = -max_border_x/2; i <= max_border_x/2; i++)
    {
        vector<point> temp;
        for 
            (int j = -max_border_y/2; j <= max_border_y/2; j++)
        {
            pt.x(static_cast<float>(i));
            pt.y(static_cast<float>(j));
            // grid[i][j] = pt;
            temp.push_back(pt);
            // cout << "i: " << i << ", j: " << j << endl;
        }
        grid.push_back(temp);
    }
}

void Grid::print_grid
()
{
    for 
        (size_t i = 0; i < grid.size(); i++)
    {
        for 
            (size_t j = 0; j < grid[i].size(); j++)
        {
            cout << "x: " << grid[i][j].x() << ", y: " << grid[i][j].y() << endl;
        }
        cout << "-----------------------\n";
    }
}

// ObstacleTypes

geometry_msgs::msg::Polygon ObstacleTypes::boost2ros
(boost::geometry::model::polygon<point> inp)
{
    geometry_msgs::msg::Polygon ret;
    for
    (const auto& point : inp.outer() )
    {

        geometry_msgs::msg::Point32 temp;
        temp.x = point.x();
        temp.y = point.y();
        ret.points.push_back(temp);
    }
    return ret;
}

boost::geometry::model::polygon<point> ObstacleTypes::ros2boost
(geometry_msgs::msg::Polygon inp)
{
    boost::geometry::model::polygon<point> ret;

    for 
    (size_t i = 0; i < inp.points.size(); i++)
    {
        point temp;
        temp.x(inp.points[i].x);
        temp.y(inp.points[i].y);
        ret.outer().push_back(temp);
    }
    return ret;
}

void ObstacleTypes::get_boost_poly
()
{
    boost_poly = ros2boost(ros_poly);
}

void ObstacleTypes::get_ros_poly
()
{
    ros_poly = boost2ros(boost_poly);
}

bool operator==(const point& lhs, const point& rhs)
{
    return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

// Util fns

float round_up
    (float inp, int places)
{
    int no = pow(10,places);

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

void test_link
    ()
{
    cout << "Connected\n";
}