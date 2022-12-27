#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <iostream>

using namespace boost::geometry;

int main()
{
    typedef boost::geometry::model::d2::point_xy<double> point;
    typedef boost::geometry::model::polygon<point> polygon;
    typedef boost::geometry::model::box<point> box;

    point p(-5,13);
    point p1(0,0);
    point p2(0,10);
    point p3(10,10);
    point p4(10,0);

    box b_box;

    std::vector<point> points = {p1, p2, p3, p4, p1};
    // Declare strategies
    const double buffer_distance = 5.0;
    const int points_per_circle = 16;
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(5.0);
    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    // Declare output
    boost::geometry::model::multi_polygon<polygon> result;

    // Declare/fill a multi point
    polygon  mp;
    polygon mp_out;
    boost::geometry::assign_points(mp,points);
    // boost::geometry::read_wkt("POLYGON((0 0,0 10,10 10,10 0,0 0))", mp);

    // Create the buffer of a multi point
    boost::geometry::buffer(mp, result,
                distance_strategy, side_strategy,
                join_strategy, end_strategy, circle_strategy);

    std::cout
        << "return_envelope:"
        << boost::geometry::dsv(boost::geometry::return_envelope<box>(result))
        << std::endl;

    b_box = boost::geometry::return_envelope<box>(result);
    std::array<point, 4> cornerPoints{{
    b_box.min_corner(),
    {b_box.max_corner().x(), b_box.min_corner().y()},
    b_box.max_corner(),
    {b_box.min_corner().x(), b_box.max_corner().y()},
  }};

    for(size_t i = 0; i < cornerPoints.size();++i)
    {
        std::cout << cornerPoints[i].x() << ", " << cornerPoints[i].y() << std::endl;
    }
    // boost::geometry::return_envelope<box>(b_box,result);

    std::cout << "Original: " << boost::geometry::wkt(mp) << std::endl;
    std::cout << "Modified: " << boost::geometry::wkt(result) << std::endl;
    std::cout << "P within: " << std::boolalpha << boost::geometry::within(p,result) << std::endl;
    std::cout << "Area of Original: " << boost::geometry::area(mp) << " Area of Modified: " << boost::geometry::area(result) << std::endl;
    std::cout << "Box: " << boost::geometry::wkt(b_box.min_corner()) << std::endl;
    return 0;
}