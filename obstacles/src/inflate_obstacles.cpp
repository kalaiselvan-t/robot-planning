#include <memory>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"

#include "./boost_geo.cpp"

using std::placeholders::_1;
using namespace std;

// Class to subscribe to the obstacles and publish inflated obstacles
class ObstaclesSubscriber: public rclcpp::Node
{
    public:
    ObstaclesSubscriber()
    : Node("obstacles_subscriber")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        subscription_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "obstacles", qos, std::bind(&ObstaclesSubscriber::topic_callback, this, _1));
    }

    private:
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_;
        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_1;
        rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_2;
        size_t count_;
        vector<geometry_msgs::msg::Polygon> polygon_list;
        geometry_msgs::msg::Polygon poly;

        void topic_callback
            (const obstacles_msgs::msg::ObstacleArrayMsg & msg)
        {
            this->fill_list(msg);
            this->array_publisher(this->inflate_obstacles());
            cout << "Publishing Inflated Obstacles at /inflated_obstacles\n";
        }

        void fill_list
            (const obstacles_msgs::msg::ObstacleArrayMsg & msg)
        {
            if 
                (polygon_list.size() == 0)
            {
                for 
                    (size_t i = 0; i < msg.obstacles.size(); i++)
                {
                    polygon_list.push_back(msg.obstacles[i].polygon);
                }
            }
            else
            {
                for
                (size_t i = 0; i < msg.obstacles.size(); i++)
                {
                    polygon_list.erase(polygon_list.begin()+i);
                    polygon_list.insert( polygon_list.begin()+i,msg.obstacles[i].polygon);
                }
            }
        }

        vector<obstacles_msgs::msg::ObstacleMsg> inflate_obstacles
            ()
        {
            std::vector<obstacles_msgs::msg::ObstacleMsg> temp_obs;

            for 
                (size_t i = 0; i < polygon_list.size(); i++)
            {
                polygon inpPoly;
                geometry_msgs::msg::Polygon poly;
                obstacles_msgs::msg::ObstacleMsg obs;
                boost::geometry::model::multi_polygon<polygon> result;

                // vector<point> points;
                // for 
                //     (size_t j = 0; j < polygon_list[i].points.size(); j++)
                // {
                //     point p(polygon_list[i].points[j].x,polygon_list[i].points[j].y);
                //     points.push_back(p);

                //     if
                //         (j == polygon_list[i].points.size())
                //     {
                //         points.push_back(point(polygon_list[i].points[0].x,polygon_list[i].points[0].y));
                //     }
                // }
                
                point p1(polygon_list[i].points[0].x,polygon_list[i].points[0].y);
                point p2(polygon_list[i].points[1].x,polygon_list[i].points[1].y);
                point p3(polygon_list[i].points[2].x,polygon_list[i].points[2].y);
                point p4(polygon_list[i].points[3].x,polygon_list[i].points[3].y);

                vector<point> points = {p1, p2, p3, p4, p1};
                
                boost::geometry::assign_points(inpPoly,points);
                std::cout << "Original: " << boost::geometry::wkt(inpPoly) << std::endl;
                inpPoly = inflate_obs(inpPoly);
                std::cout << "Modified: " << boost::geometry::wkt(inpPoly) << std::endl;
                
                vector<geometry_msgs::msg::Point32> inf_points;

                for
                    ( const auto& point : inpPoly.outer() )
                {

                    geometry_msgs::msg::Point32 temp;
                    temp.x = point.x();
                    temp.y = point.y();
                    inf_points.push_back(temp);
                }
                poly.points = inf_points;
                obs.polygon = poly;
                temp_obs.push_back(obs);
            }
            return  temp_obs;
        }

        void array_publisher
            (vector<obstacles_msgs::msg::ObstacleMsg> inp)
        {
            pub_ = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("inflated_obstacles", 10);
            pub_1 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("inf_obs1", 10);
            pub_2 = this->create_publisher<geometry_msgs::msg::PolygonStamped>("inf_obs2", 10);
            std_msgs::msg::Header hh;
            hh.stamp = this->get_clock()->now();
            hh.frame_id = "map";
            obstacles_msgs::msg::ObstacleArrayMsg msg;
            geometry_msgs::msg::PolygonStamped pol1;
            geometry_msgs::msg::PolygonStamped pol2;

            pol1.header = hh;
            pol2.header = hh;
            pol1.polygon = inp[0].polygon;
            pol2.polygon = inp[1].polygon;
            msg.obstacles = inp;

            try
            {
                if
                    (rclcpp::ok())
                {
                    pub_1->publish(pol1);
                    pub_2->publish(pol2);
                    pub_->publish(msg);
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }

        void print
            ()
        {
            cout << "Polygon list: \n";
            cout << "size: " << polygon_list.size() << endl;

            for 
                (size_t i = 0; i < polygon_list.size(); i++)
            {
                for 
                    (size_t j = 0; j < polygon_list[i].points.size(); j++)
                {
                    cout << "x: " << polygon_list[i].points[j].x << " y: " << polygon_list[i].points[j].y << endl;
                }
                   
            }
            cout << "size: " << polygon_list.size() << endl;
            cout << "=======================\n"; 
        }
};


int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstaclesSubscriber>());
    rclcpp::shutdown();
    return 0;
}