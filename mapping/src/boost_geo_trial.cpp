#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <iostream>

using namespace boost::geometry;

int main()
{
    typedef double blah;
    typedef boost::geometry::model::d2::point_xy<blah> point;
    typedef boost::geometry::model::polygon<point> polygon;

    point p(-5,13);
    point p1(0,0);
    point p2(0,10);
    point p3(10,10);
    point p4(10,0);
    

    std::vector<point> points = {p1, p2, p3, p4, p1};
    // Declare strategies
    const double buffer_distance = 5.0;
    const int points_per_circle = 16;
    boost::geometry::strategy::buffer::distance_symmetric<blah> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    // Declare output
    boost::geometry::model::multi_polygon<polygon> result;

    // Declare/fill a multi point
    boost::geometry::model::polygon<point> mp;
    polygon mp_out;
    boost::geometry::assign_points(mp,points);
    // boost::geometry::read_wkt("POLYGON((0 0,10 0,10 10,0 10,0 0))", mp);
    // boost::geometry::read_wkt("POLYGON((0 0,0 10,10 10,10 0,0 0))", mp);
    // boost::geometry::read_wkt("MULTIPOINT((3 3),(4 4),(6 2))", mp);

    // Create the buffer of a multi point
    boost::geometry::buffer(mp, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, circle_strategy);

    

    std::cout << "Original: " << boost::geometry::wkt(mp) << std::endl;
    std::cout << "Modified: " << boost::geometry::wkt(result) << std::endl;
    std::cout << "P within: " << std::boolalpha << boost::geometry::within(p,result) << std::endl;
    std::cout << "Area of Original: " << boost::geometry::area(mp) << " Area of Modified: " << boost::geometry::area(result) << std::endl;
    // boost::geometry::convert(result,mp_out);
    // boost::geometry::assign(mp,mp_out);
    // std::cout << "Inflated: " << boost::geometry::wkt(mp_out) << std::endl;
    // std::cout << "Number of segments: " << boost::geometry::num_segments(mp) << std::endl;
    // std::cout << "envelope:" << boost::geometry::dsv(result) << std::endl;
    // std::cout << boost::geometry::get<boost::geometry::min_corner, 0>(result);
    std::cout << "Original: " << boost::geometry::wkt(boost::geometry::exterior_ring(result)) << std::endl;
    return 0;
}