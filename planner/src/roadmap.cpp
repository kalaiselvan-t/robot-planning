#include "../include/planner/r_map.h"

using std::placeholders::_1;

typedef geometry_msgs::msg::Point32 ros_point;
// std::signal(SIGSEGV, handler);

random_device rd;
mt19937 gen(rd());

uniform_real_distribution<float> dis_x(0,max_border_x);
uniform_real_distribution<float> dis_y(0, max_border_y);

class RoadMap : public rclcpp::Node
{
    public:
        RoadMap():Node("roadmap_node")
        {
            sub_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
                "/inflated_obstacles",1,bind(&RoadMap::topic_callback, this, _1));

            pub_ = this->create_publisher<obstacles_msgs::msg::WaypointsMsg>("waypoints",10);
        }

    private:
        rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr sub_;
        rclcpp::Publisher<obstacles_msgs::msg::WaypointsMsg>::SharedPtr pub_;
        // rclcpp::TimerBase::SharedPtr timer_;
        // size_t count_;

        vector<pair<Point,pair<int,int>>> follow_path;
        Grid dup_grid;

        void topic_callback
        (const obstacles_msgs::msg::ObstacleArrayMsg & msg)
        {
            cout << "===========Beginning of DEBUG=============\n\n";
            follow_path.clear();
            if
            (obs_list.size() == 0)
            {
                create_poly_list(msg);
            }
            create_node_graph();
            // cout << "===========End of graph=============\n";
            remove_obs_in_grid();
            dup_grid = map_grid;
            // map_grid.print_grid();
            print_obs();
            ros_point st,en;
            st.x = 0.0;
            st.y = 0.0;
            en.x = 2.0;
            en.y = 2.0;
            find_path(st,en);
            print_obs_pts();
            print_followpath();
            cout << "\n===========End of graph=============\n";
            publisher();
            // map_grid.print_grid();
            // cout << "Receiving data\n";
            // print_followpath();
        }

        void publisher
        ()
        {
            obstacles_msgs::msg::WaypointsMsg w_msg;
            vector<geometry_msgs::msg::Point32> way_pts_list;
            
            for (size_t i = 0; i < follow_path.size(); i++)
            {
                w_msg.waypoints.push_back(follow_path[i].first.ros_pt);
            }
            pub_->publish(w_msg);
            cout << "waypoints published\n";
        }

        void create_poly_list
        (const obstacles_msgs::msg::ObstacleArrayMsg & msg)
        {
            for 
            (size_t i = 0; i < msg.obstacles.size(); i++)
            {
                ObstacleTypes obs;
                obs.ros_poly = msg.obstacles[i].polygon;
                for (size_t l = 0; l < obs.ros_poly.points.size(); l++)
                // {
                //     pair<float,float> pt;
                //     pt.first = obs.ros_poly.points[l].x;
                //     pt.second = obs.ros_poly.points[l].y;
                //     obs_pts_list.push_back(pt);
                // }
                obs.get_boost_poly();
                obs_list.push_back(obs);
            }
        }

        void remove_obs_in_grid
        ()
        {   
            
            for 
                (size_t i = 0; i < map_grid.grid.size(); i++)
            {
                for 
                    (size_t j = 0; j < map_grid.grid[i].size(); j++)
                {
                    // for 
                    // (size_t m = 0; m < obs_pts_list.size(); m++)
                    // {
                    //     if(map_grid.grid[i][j].x() == obs_pts_list[m].first && map_grid.grid[i][j].y() == obs_pts_list[m].second)
                    //     {
                    //         map_grid.grid[i][j].x(std::numeric_limits<float>::infinity());
                    //         map_grid.grid[i][j].y(std::numeric_limits<float>::infinity());
                    //     }
                    // }
                    
                    for 
                    (size_t k = 0; k < obs_list.size(); k++)
                    {
                        if(boost::geometry::covered_by(map_grid.grid[i][j], obs_list[k].boost_poly))
                        {
                            cout << "inside obs: " << map_grid.grid[i][j].x() << ", " << map_grid.grid[i][j].y() << endl;
                            // auto it = std::find(map_grid.grid[i].begin(),map_grid.grid[i].end(),map_grid.grid[i][j]);
                            // map_grid.grid[i].erase(it);
                            // map_grid.grid[i].erase(map_grid.grid[i].begin()+j);
                            map_grid.grid[i][j].x(std::numeric_limits<float>::infinity());
                            map_grid.grid[i][j].y(std::numeric_limits<float>::infinity());
                            // cout << "new_x : " << map_grid.grid[i][j].x() << " ,new_y: " << map_grid.grid[i][j].y() << endl;
                            // cout << isinf(map_grid.grid[i][j].x()) << endl;
                        }
                    }
                } 
            }  
        }

        NodeGraph create_random_node
        ()
        {
            NodeGraph node;
            float x = round_up(dis_x(gen),2);
            float y = round_up(dis_y(gen),2);

            for (size_t i = 0; i < obs_list.size(); i++)
            {
                if(boost::geometry::covered_by(point(x,y),obs_list[i].boost_poly))
                {
                    create_random_node();
                }
                else
                {
                    node.x = x;
                    node.y = y;
                    return node;
                }
            }
        }

        void create_node_graph
        ()
        {
            while 
            (static_cast<int>(roadmap_nodes.list.size()) < resolution)
            {
                roadmap_nodes.list.push_back(create_random_node());
            }
            // roadmap_nodes.print();
        }

        void find_path
            (geometry_msgs::msg::Point32 st, geometry_msgs::msg::Point32 en)
        {
            Point start(st);
            Point end(en);
            pair<int,int> start_ind, end_ind;

            for (size_t i = 0; i < map_grid.grid.size(); i++)
            {
                for (size_t j = 0; j < map_grid.grid[i].size(); j++)
                {
                    if (start.boost_pt == map_grid.grid[i][j])
                    {
                        start_ind.first = i;
                        start_ind.second = j;
                    }

                    if (end.boost_pt == map_grid.grid[i][j])
                    {
                        end_ind.first = i;
                        end_ind.second = j;
                    }
                }
            }

            cout << start_ind.first << ", " << start_ind.second << endl;
            // cout << map_grid.grid[15][13].x() << ", " << map_grid.grid[15][13].y() << endl;

            if(follow_path.size() == 0)
            {
                pair<Point,pair<int,int>> temp;
                temp.first = start;
                temp.second = start_ind;
                follow_path.push_back(temp);
            }
            // cout << follow_path[0].second.first << ", " << follow_path[0].second.second << endl;
            // else
            // {
                int count = 0;
                while(!follow_path[follow_path.size()-1].first.operator==(end.boost_pt))
                {   
                    if(count > 0)
                    {
                        dup_grid.grid[follow_path[count-1].second.first][follow_path[count-1].second.second].x(std::numeric_limits<float>::infinity());
                        dup_grid.grid[follow_path[count-1].second.first][follow_path[count-1].second.second].y(std::numeric_limits<float>::infinity());
                    }
                    pair<Point,pair<int,int>> best;
                    float best_dist = std::numeric_limits<float>::infinity();

                    //up
                    int up_x = follow_path[follow_path.size()-1].second.first;
                    int up_y = follow_path[follow_path.size()-1].second.second + 1;
                    cout << "up_x: " << up_x << ", " << " up_y: " << up_y << endl;
                    cout << "x: " << dup_grid.grid[up_x][up_y].x() << ", y: " << dup_grid.grid[up_x][up_y].y() << endl;
                    
                    if (0 <= up_x && up_x <= max_border_x && 0 <= up_y && up_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[up_x][up_y].x()) || isinf(dup_grid.grid[up_x][up_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[up_x][up_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[up_x][up_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[up_x][up_y]);
                                best.second = {up_x,up_y};
                            }
                            else
                            {
                                dup_grid.grid[up_x][up_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[up_x][up_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                            
                        }
                        else{
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }
                    
                    
                    //up right
                    
                    int up_right_x = follow_path.back().second.first + 1;
                    int up_right_y = follow_path.back().second.second + 1;
                    cout << "up_right_x: " << up_right_x << ", " << " up_right_y: " << up_right_y << endl;
                    cout << "x: " << dup_grid.grid[up_right_x][up_right_y].x() << ", y: " << dup_grid.grid[up_right_x][up_right_y].y() << endl;
                    
                    if (0 <= up_right_x && up_right_x <= max_border_x && 0 <= up_right_y && up_right_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[up_right_x][up_right_y].x()) || isinf(dup_grid.grid[up_right_x][up_right_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[up_right_x][up_right_y],end.boost_pt) <= best_dist)
                            {   
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[up_right_x][up_right_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[up_right_x][up_right_y]);
                                best.second = {up_right_x,up_right_y};
                            }
                            else
                            {
                                dup_grid.grid[up_right_x][up_right_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[up_right_x][up_right_y].y(std::numeric_limits<float>::infinity());
                            }
                            
                            cout << "not a obs\n";
                        }
                        else
                        {
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }
                    

                    //right
                    int right_x = follow_path.back().second.first + 1;
                    int right_y = follow_path.back().second.second;
                    cout << "right_x: " << right_x << ", " << " right_y: " << right_y << endl;
                    cout << "x: " << dup_grid.grid[right_x][right_y].x() << ", y: " << dup_grid.grid[right_x][right_y].y() << endl;
                      
                    if (0 <= right_x && right_x <= max_border_x && 0 <= right_y && right_y <= max_border_y)
                    {
                        if(!( isinf(dup_grid.grid[right_x][right_y].x()) || isinf(dup_grid.grid[right_x][right_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[right_x][right_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[right_x][right_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[right_x][right_y]);
                                best.second = {right_x,right_y};
                            }
                            else
                            {
                                dup_grid.grid[right_x][right_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[right_x][right_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                        }
                        else
                        {
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }

                    // right down
                    int right_down_x = follow_path.back().second.first + 1;
                    int right_down_y = follow_path.back().second.second - 1;
                    cout << "right_down_x: " << right_down_x << ", " << " right_down_y: " << right_down_y << endl;
                    cout << "x: " << dup_grid.grid[right_down_x][right_down_y].x() << ", y: " << dup_grid.grid[right_down_x][right_down_y].y() << endl;
                     
                    if (0 <= right_down_x && right_down_x <= max_border_x && 0 <= right_down_y && right_down_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[right_down_x][right_down_y].x()) || isinf(dup_grid.grid[right_down_x][right_down_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[right_down_x][right_down_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[right_down_x][right_down_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[right_down_x][right_down_y]);
                                best.second = {right_down_x,right_down_y};
                            }
                            else
                            {
                                dup_grid.grid[right_down_x][right_down_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[right_down_x][right_down_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                        }
                        else
                        {
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }

                    // down
                    int down_x = follow_path.back().second.first;
                    int down_y = follow_path.back().second.second - 1;
                    cout << "down_x: " << down_x << ", " << " down_y: " << down_y << endl;
                    cout << "x: " << dup_grid.grid[down_x][down_y].x() << ", y: " << dup_grid.grid[down_x][down_y].y() << endl;
                       
                    if (0 <= down_x && down_x <= max_border_x && 0 <= down_y && down_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[down_x][down_y].x()) || isinf(dup_grid.grid[down_x][down_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[down_x][down_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[down_x][down_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[down_x][down_y]);
                                best.second = {down_x,down_y};
                            }
                            else
                            {
                                dup_grid.grid[down_x][down_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[down_x][down_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                        }
                        else
                        {
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }

                    // down left
                    int down_left_x = follow_path.back().second.first - 1;
                    int down_left_y = follow_path.back().second.second - 1;
                    cout << "down_left_x: " << down_left_x << ", " << " down_left_y: " << down_left_y << endl;
                    cout << "x: " << dup_grid.grid[down_left_x][down_left_y].x() << ", y: " << dup_grid.grid[down_left_x][down_left_y].y() << endl;
                     
                    if (0 <= down_left_x && down_left_x <= max_border_x && 0 <= down_left_y && down_left_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[down_left_x][down_left_y].x()) || isinf(dup_grid.grid[down_left_x][down_left_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[down_left_x][down_left_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[down_left_x][down_left_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[down_left_x][down_left_y]);
                                best.second = {down_left_x,down_left_y};
                            }
                            else
                            {
                                dup_grid.grid[down_left_x][down_left_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[down_left_x][down_left_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                        }
                        else
                        {
                            cout << "obs\n";
                        }
                        
                        // cout << "inside bounds\n";
                    }

                    // left
                    int left_x = follow_path.back().second.first - 1;
                    int left_y = follow_path.back().second.second;
                    cout << "left_x: " << left_x << ", " << " left_y: " << left_y << endl;
                    cout << "x: " << dup_grid.grid[left_x][left_y].x() << ", y: " << dup_grid.grid[left_x][left_y].y() << endl;
                    
                    if (0 <= left_x && left_x <= max_border_x && 0 <= left_y && left_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[left_x][left_y].x()) || isinf(dup_grid.grid[left_x][left_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[left_x][left_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[left_x][left_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[left_x][left_y]);
                                best.second = {left_x,left_y};
                            }
                            else
                            {
                                dup_grid.grid[left_x][left_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[left_x][left_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                        }
                        else{
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }
                    

                    // left up
                    int left_up_x = follow_path.back().second.first - 1;
                    int left_up_y = follow_path.back().second.second + 1;
                    cout << "left_up_x: " << left_up_x << ", " << " left_up_y: " << left_up_y << endl;
                    cout << "x: " << dup_grid.grid[left_up_x][left_up_y].x() << ", y: " << dup_grid.grid[left_up_x][left_up_y].y() << endl;
                      
                    if (0 <= left_up_x && left_up_x <= max_border_x && 0 <= left_up_y && left_up_y <= max_border_y)
                    {
                        if(!(isinf(dup_grid.grid[left_up_x][left_up_y].x()) || isinf(dup_grid.grid[left_up_x][left_up_y].y())))
                        {
                            if(boost::geometry::distance(dup_grid.grid[left_up_x][left_up_y],end.boost_pt) <= best_dist)
                            {
                                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                                best_dist = boost::geometry::distance(dup_grid.grid[left_up_x][left_up_y],end.boost_pt);
                                best.first = Point(dup_grid.grid[left_up_x][left_up_y]);
                                best.second = {left_up_x,left_up_y};
                            }
                            else
                            {
                                dup_grid.grid[left_up_x][left_up_y].x(std::numeric_limits<float>::infinity());
                                dup_grid.grid[left_up_x][left_up_y].y(std::numeric_limits<float>::infinity());
                            }
                            cout << "not a obs\n";
                        }
                        else
                        {
                            cout << "obs\n";
                        }
                        // cout << "inside bounds\n";
                    }
                   
                    follow_path.push_back(best);
                    count++;
                    cout << "-------------------------------------\n";
                }
            // }
        }

        void print_obs
        ()
        {
            cout << "=========Printing Obstacles=============\n";
            for 
            (size_t i = 0; i < obs_list.size(); i++)
            {
                cout << "i: " << i << endl;
                for (size_t j = 0; j < obs_list[i].ros_poly.points.size(); j++)
                {
                    cout << "x: " << obs_list[i].ros_poly.points[j].x << ", y: " <<  obs_list[i].ros_poly.points[j].y << endl;
                }
                
            }
            cout << "=========End of Obstacles=============\n";
        }

        void print_followpath()
        {   
            cout << "============FollowPath Start============\n";
            for 
            (size_t i = 0; i < follow_path.size(); i++)
            {
                cout << "i: " << i << " x: " << follow_path[i].first.boost_pt.x() << " y: " << follow_path[i].first.boost_pt.y() << endl;
            }
            cout << "============FollowPath End=============\n";
        }

        void print_obs_pts
        ()
        {
            for 
            (size_t i = 0; i < obs_pts_list.size(); i++)
            {
                cout << "x: " << obs_pts_list[i].first << ", y: " <<  obs_pts_list[i].second << endl;
            }
            
        }
};

int main(int argc, char * argv[])
{
    // while 
    //     (static_cast<int>(roadmap_nodes.list.size()) < resolution)
    // {
    //     roadmap_nodes.list.push_back(create_random_node());
    // }

    // roadmap_nodes.print();

    map_grid.create_grid();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoadMap>());
    rclcpp::shutdown();
    return 0;
}