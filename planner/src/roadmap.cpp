#include "../include/planner/r_map.h"

using std::placeholders::_1;
// std::signal(SIGSEGV, handler);

random_device rd;
mt19937 gen(rd());

uniform_real_distribution<float> dis_x(0,max_border_x);
uniform_real_distribution<float> dis_y(0, max_border_y);


void Roadmap::topic_callback(const obstacles_msgs::msg::ObstacleArrayMsg & msg)
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
    ros_point start, end;
    start.x = 2.0;
    start.y = 2.0;
    end.x = -4.0;
    end.y = -4.0;
    bool ok = start_condition(start, end);

    if(ok){
        // find_path(st,en);
        print_obs_pts();
        // print_followpath();
        cout << "\n===========End of graph=============\n";
        // publisher();
    }
    else{
        cout << "start or end is inside obs\n";
    }
    // map_grid.print_grid();
    // cout << "Receiving data\n";
    // print_followpath();
}

bool Roadmap::start_condition(ros_point start, ros_point end)
{
    bool ret = true;
    if(start != end){
        for (size_t i = 0; i < obs_list.size(); i++)
        {
            obs_list[i].get_boost_poly();
            if(boost::geometry::covered_by(boost_point(start.x, start.y), obs_list[i].boost_poly) || boost::geometry::covered_by(boost_point(end.x, end.y), obs_list[i].boost_poly))
            {
                cout << "start or end is inside obs\n";
                ret = false;
            }
        }
    }
    cout << "start status: " << ret << endl;

    return ret;
}

void Roadmap::find_path(ros_point start, ros_point end, vector<pair<Point,pair<int,int>>> &fp)
{
    Point point_start(start);
    Point point_end(end);
    pair<int,int> start_ind, end_ind;

    for (size_t i = 0; i < map_grid.grid.size(); i++)
    {
        for (size_t j = 0; j < map_grid.grid[i].size(); j++)
        {
            if (point_start.boost_pt == map_grid.grid[i][j])
            {
                start_ind.first = i;
                start_ind.second = j;
            }

            if (point_end.boost_pt == map_grid.grid[i][j])
            {
                end_ind.first = i;
                end_ind.second = j;
            }
        }
    }

    cout << start_ind.first << ", " << start_ind.second << endl;
    // cout << map_grid.grid[15][13].x() << ", " << map_grid.grid[15][13].y() << endl;

    if(fp.size() == 0)
    {
        pair<Point,pair<int,int>> temp;
        temp.first = start;
        temp.second = start_ind;
        fp.push_back(temp);
    }
    // cout << follow_path[0].second.first << ", " << follow_path[0].second.second << endl;
    // else
    // {
        int count = 0;
        while(!fp[fp.size()-1].first.operator==(point_end.boost_pt) && count < 100)
        {   
            if(count > 0)
            {
                dup_grid.grid[fp[count-1].second.first][fp[count-1].second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[fp[count-1].second.first][fp[count-1].second.second].y(std::numeric_limits<float>::infinity());
            }

            pair<Point,pair<int,int>> best;
            float best_dist = std::numeric_limits<float>::infinity();

            //up
            up(end, best, best_dist, fp);
            
            //up right
            up_right(end, best, best_dist, fp);

            //right
            right(end, best, best_dist, fp);

            // right down
            right_down(end, best, best_dist, fp);

            // down
            down(end, best, best_dist, fp);

            // down left
            down_left(end, best, best_dist, fp);

            // left
            left(end, best, best_dist, fp);
            

            // left up
            left_up(end, best, best_dist, fp);
            
            // cout << "x: " << best.first.boost_pt.x() << ", y: " << best.first.boost_pt.y() << endl;
            fp.push_back(best);
            count++;
            cout << "-------------------------------------\n";
        }
    // }
}

// void Roadmap::publisher()
// {
//     obstacles_msgs::msg::WaypointsMsg w_msg;
//     vector<geometry_msgs::msg::Point32> way_pts_list;
    
//     for (size_t i = 0; i < fp.size(); i++)
//     {
//         w_msg.waypoints.push_back(fp[i].first.ros_pt);
//     }
//     pub_->publish(w_msg);
//     cout << "waypoints published\n";
// }

void Roadmap::create_poly_list(const obstacles_msgs::msg::ObstacleArrayMsg & msg)
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

void Roadmap::remove_obs_in_grid()
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

NodeGraph Roadmap::create_random_node()
{
    NodeGraph node;
    float x = round_up2(dis_x(gen),2);
    float y = round_up2(dis_y(gen),2);

    for (size_t i = 0; i < obs_list.size(); i++)
    {
        if(boost::geometry::covered_by(boost_point(x,y),obs_list[i].boost_poly))
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

void Roadmap::create_node_graph()
{
    while 
    (static_cast<int>(roadmap_nodes.list.size()) < resolution)
    {
        roadmap_nodes.list.push_back(create_random_node());
    }
    // roadmap_nodes.print();
}

void Roadmap::up(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int up_x = fp[fp.size()-1].second.first;
    int up_y = fp[fp.size()-1].second.second + 1;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl; 
    cout << "up_x: " << up_x << ", " << " up_y: " << up_y << endl;
    
    if (0 <= up_x && up_x <= max_border_x && 0 <= up_y && up_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[up_x][up_y].x() << ", y: " << dup_grid.grid[up_x][up_y].y() << endl;
        if(!(isinf(dup_grid.grid[up_x][up_y].x()) || isinf(dup_grid.grid[up_x][up_y].y())) && abs(boost::geometry::distance(dup_grid.grid[up_x][up_y],current_pt)) < 5)
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
    }
}

void Roadmap::up_right(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int up_right_x = fp.back().second.first + 1;
    int up_right_y = fp.back().second.second + 1;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "up_right_x: " << up_right_x << ", " << " up_right_y: " << up_right_y << endl;
    
    if (0 <= up_right_x && up_right_x <= max_border_x && 0 <= up_right_y && up_right_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[up_right_x][up_right_y].x() << ", y: " << dup_grid.grid[up_right_x][up_right_y].y() << endl;
        if(!(isinf(dup_grid.grid[up_right_x][up_right_y].x()) || isinf(dup_grid.grid[up_right_x][up_right_y].y())) && abs(boost::geometry::distance(dup_grid.grid[up_right_x][up_right_y],current_pt)) < 5)
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
    }
}

void Roadmap::right(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int right_x = fp.back().second.first + 1;
    int right_y = fp.back().second.second;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "right_x: " << right_x << ", " << " right_y: " << right_y << endl;
        
    if (0 <= right_x && right_x <= max_border_x && 0 <= right_y && right_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[right_x][right_y].x() << ", y: " << dup_grid.grid[right_x][right_y].y() << endl;
        if(!(isinf(dup_grid.grid[right_x][right_y].x()) || isinf(dup_grid.grid[right_x][right_y].y())) && abs(boost::geometry::distance(dup_grid.grid[right_x][right_y],current_pt)) < 5)
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
    }
}

void Roadmap::right_down(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int right_down_x = fp.back().second.first + 1;
    int right_down_y = fp.back().second.second - 1;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "right_down_x: " << right_down_x << ", " << " right_down_y: " << right_down_y << endl;
        
    if (0 <= right_down_x && right_down_x <= max_border_x && 0 <= right_down_y && right_down_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[right_down_x][right_down_y].x() << ", y: " << dup_grid.grid[right_down_x][right_down_y].y() << endl;
        if(!(isinf(dup_grid.grid[right_down_x][right_down_y].x()) || isinf(dup_grid.grid[right_down_x][right_down_y].y())) && abs(boost::geometry::distance(dup_grid.grid[right_down_x][right_down_y],current_pt)) < 5)
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
    }
}

void Roadmap::down(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int down_x = fp.back().second.first;
    int down_y = fp.back().second.second - 1;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "down_x: " << down_x << ", " << " down_y: " << down_y << endl;
        
    if (0 <= down_x && down_x <= max_border_x && 0 <= down_y && down_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[down_x][down_y].x() << ", y: " << dup_grid.grid[down_x][down_y].y() << endl;
        if(!(isinf(dup_grid.grid[down_x][down_y].x()) || isinf(dup_grid.grid[down_x][down_y].y())) && abs(boost::geometry::distance(dup_grid.grid[down_x][down_y],current_pt)) < 5)
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
    }
}

void Roadmap::down_left(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int down_left_x = fp.back().second.first - 1;
    int down_left_y = fp.back().second.second - 1;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "down_left_x: " << down_left_x << ", " << " down_left_y: " << down_left_y << endl;
        
    if (0 <= down_left_x && down_left_x <= max_border_x && 0 <= down_left_y && down_left_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[down_left_x][down_left_y].x() << ", y: " << dup_grid.grid[down_left_x][down_left_y].y() << endl;
        if(!(isinf(dup_grid.grid[down_left_x][down_left_y].x()) || isinf(dup_grid.grid[down_left_x][down_left_y].y())) && abs(boost::geometry::distance(dup_grid.grid[down_left_x][down_left_y],current_pt)) < 5)
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
    }
}

void Roadmap::left(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int left_x = fp.back().second.first - 1;
    int left_y = fp.back().second.second;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "left_x: " << left_x << ", " << " left_y: " << left_y << endl;
    
    if (0 <= left_x && left_x <= max_border_x && 0 <= left_y && left_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[left_x][left_y].x() << ", y: " << dup_grid.grid[left_x][left_y].y() << endl;
        if(!(isinf(dup_grid.grid[left_x][left_y].x()) || isinf(dup_grid.grid[left_x][left_y].y())) && abs(boost::geometry::distance(dup_grid.grid[left_x][left_y],current_pt)) < 5)
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
    }
}

void Roadmap::left_up(Point end, pair<Point,pair<int,int>> &best,float &best_dist, vector<pair<Point,pair<int,int>>> &fp)
{
    int left_up_x = fp.back().second.first - 1;
    int left_up_y = fp.back().second.second + 1;
    boost_point current_pt = fp[fp.size()-1].first.boost_pt;
    cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    cout << "left_up_x: " << left_up_x << ", " << " left_up_y: " << left_up_y << endl;
        
    if (0 <= left_up_x && left_up_x <= max_border_x && 0 <= left_up_y && left_up_y <= max_border_y)
    {
        cout << "x: " << dup_grid.grid[left_up_x][left_up_y].x() << ", y: " << dup_grid.grid[left_up_x][left_up_y].y() << endl;
        if(!(isinf(dup_grid.grid[left_up_x][left_up_y].x()) || isinf(dup_grid.grid[left_up_x][left_up_y].y())) && abs(boost::geometry::distance(dup_grid.grid[left_up_x][left_up_y],current_pt)) < 5)
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
    }
}

void Roadmap::print_obs()
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

void Roadmap::print_followpath(vector<pair<Point,pair<int,int>>> &fp)
{
    cout << "============FollowPath Start============\n";
    for 
    (size_t i = 0; i < fp.size(); i++)
    {
        cout << "i: " << i << " x: " << fp[i].first.boost_pt.x() << " y: " << fp[i].first.boost_pt.y() << endl;
    }
    cout << "============FollowPath End=============\n";
}

void Roadmap::print_obs_pts()
{
    for 
    (size_t i = 0; i < obs_pts_list.size(); i++)
    {
        cout << "x: " << obs_pts_list[i].first << ", y: " <<  obs_pts_list[i].second << endl;
    }    
}