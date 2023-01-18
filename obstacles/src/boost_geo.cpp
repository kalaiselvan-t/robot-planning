#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <iostream>
#include <vector>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;
typedef boost::geometry::model::box<point> box;

const double buffer_distance = 0.25;
const int points_per_circle = 16;
boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance);
boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
boost::geometry::strategy::buffer::side_straight side_strategy;

polygon inflate_obs
    (polygon inp)
{
    boost::geometry::model::multi_polygon<polygon> result;
    box b_box;
    boost::geometry::buffer(inp, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, circle_strategy);

    b_box = boost::geometry::return_envelope<box>(result);
    std::vector<point> cornerPoints{{
        b_box.min_corner(),
        {b_box.max_corner().x(), b_box.min_corner().y()},
        b_box.max_corner(),
        {b_box.min_corner().x(), b_box.max_corner().y()},
    }};
    
    boost::geometry::assign_points(inp,cornerPoints);

    return inp;
}