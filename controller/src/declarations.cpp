#include "../include/controller/common.hpp"
#include "../include/controller/data.hpp"
#include "./dubins.cpp"
/*
========================================================POINT=====================================================
*/

Point::Point(geometry_msgs::msg::Pose inp)
{
    ros_pose = inp;
    get_boost_pt();
}

Point::Point(boost_point inp)
{
    boost_pt = inp;
    get_ros_pose();
}

Point::Point()
{
}

void Point::get_boost_pt()
{
    boost_pt.x(ros_pose.position.x);
    boost_pt.y(ros_pose.position.y);
}

void Point::get_ros_pose()
{
    ros_pose.position.x = boost_pt.x();
    ros_pose.position.y = boost_pt.y();
}

/*
========================================================GRID=====================================================
*/
void Grid::create_grid()
{
    for (float i = -max_border_x / 2; i <= max_border_x / 2; i++)
    {
        vector<boost_point> temp;
        for (float j = -max_border_y / 2; j <= max_border_y / 2; j++)
        {
            boost_point p;
            p.x(static_cast<float>(i));
            p.y(static_cast<float>(j));
            temp.push_back(p);
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

/*
========================================================OBSTACLE TYPES=====================================================
*/

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

/*
========================================================PLANNER CLASS=====================================================
*/
Planner::Planner(geometry_msgs::msg::Pose start, geometry_msgs::msg::Pose end, Grid map)
{
    this->start = start;
    this->end = end;
    this->dup_grid = map;
}

void Planner::find_path()
{
    Point st(start);
    Point en(end);
    // Point start(start.ros_pose);
    // Point end(end.ros_pose);
    // cout << "st_boost_x: " << st.boost_pt.x() << ", st_boost_y: " << st.boost_pt.x() << endl;
    pair<int,int> start_ind, end_ind;

    // cout << "grid size: " << this->dup_grid.grid.size() << endl;

    for (size_t i = 0; i < this->dup_grid.grid.size(); i++)
    {
        for (size_t j = 0; j < this->dup_grid.grid[i].size(); j++)
        {
            if (st.boost_pt == this->dup_grid.grid[i][j])
            {
                start_ind.first = i;
                start_ind.second = j;
            }

            if (en.boost_pt == this->dup_grid.grid[i][j])
            {
                end_ind.first = i;
                end_ind.second = j;
            }
        }
    }

    cout << start_ind.first << ", " << start_ind.second << endl;

    if(this->follow_path.size() == 0)
    {
        pair<Point,pair<int,int>> temp;
        temp.first = st;
        temp.second = start_ind;
        this->follow_path.push_back(temp);
        // cout << "follow path size: 0\n";
    }
    // cout << this->follow_path[0].second.first << ", " << this->follow_path[0].second.second << endl;
    
    int count = 0;
    while(!follow_path[follow_path.size()-1].first.operator==(en.boost_pt) && count < 100)
    {   
        if(count > 0)
        {
            dup_grid.grid[follow_path[count-1].second.first][follow_path[count-1].second.second].x(std::numeric_limits<float>::infinity());
            dup_grid.grid[follow_path[count-1].second.first][follow_path[count-1].second.second].y(std::numeric_limits<float>::infinity());
        }

        pair<Point,pair<int,int>> best;
        float best_dist = std::numeric_limits<float>::infinity();

        //up
        up(en, best, best_dist);
        
        //up right
        up_right(en, best, best_dist);

        // //right
        right(en, best, best_dist);

        // // right down
        right_down(en, best, best_dist);

        // // down
        down(en, best, best_dist);

        // // down left
        down_left(en, best, best_dist);

        // // left
        left(en, best, best_dist);
        

        // // left up
        left_up(en, best, best_dist);
        
        // cout << "x: " << best.first.boost_pt.x() << ", y: " << best.first.boost_pt.y() << endl;
        follow_path.push_back(best);
        count++;
        // cout << "-------------------------------------\n";
    }
}

void Planner::up(Point en, pair<Point,pair<int,int>> &best, float &best_dist)
{
    int up_x = follow_path[follow_path.size()-1].second.first;
    int up_y = follow_path[follow_path.size()-1].second.second + 1;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl; 
    // cout << "up_x: " << up_x << ", " << " up_y: " << up_y << endl;
    
    if (0 <= up_x && up_x <= max_border_x && 0 <= up_y && up_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[up_x][up_y].x() << ", y: " << dup_grid.grid[up_x][up_y].y() << endl;
        if(!(isinf(dup_grid.grid[up_x][up_y].x()) || isinf(dup_grid.grid[up_x][up_y].y())) && abs(boost::geometry::distance(dup_grid.grid[up_x][up_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[up_x][up_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[up_x][up_y],en.boost_pt);
                best.first = Point(dup_grid.grid[up_x][up_y]);
                best.second = {up_x,up_y};
            }
            else
            {
                dup_grid.grid[up_x][up_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[up_x][up_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else{
            // cout << "obs\n";
        }
    }
}

void Planner::up_right(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int up_right_x = follow_path.back().second.first + 1;
    int up_right_y = follow_path.back().second.second + 1;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "up_right_x: " << up_right_x << ", " << " up_right_y: " << up_right_y << endl;
    
    if (0 <= up_right_x && up_right_x <= max_border_x && 0 <= up_right_y && up_right_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[up_right_x][up_right_y].x() << ", y: " << dup_grid.grid[up_right_x][up_right_y].y() << endl;
        if(!(isinf(dup_grid.grid[up_right_x][up_right_y].x()) || isinf(dup_grid.grid[up_right_x][up_right_y].y())) && abs(boost::geometry::distance(dup_grid.grid[up_right_x][up_right_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[up_right_x][up_right_y],en.boost_pt) <= best_dist)
            {   
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[up_right_x][up_right_y],en.boost_pt);
                best.first = Point(dup_grid.grid[up_right_x][up_right_y]);
                best.second = {up_right_x,up_right_y};
            }
            else
            {
                dup_grid.grid[up_right_x][up_right_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[up_right_x][up_right_y].y(std::numeric_limits<float>::infinity());
            }
            
            // cout << "not a obs\n";
        }
        else
        {
            // cout << "obs\n";
        }
    }
}

void Planner::right(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int right_x = follow_path.back().second.first + 1;
    int right_y = follow_path.back().second.second;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "right_x: " << right_x << ", " << " right_y: " << right_y << endl;
        
    if (0 <= right_x && right_x <= max_border_x && 0 <= right_y && right_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[right_x][right_y].x() << ", y: " << dup_grid.grid[right_x][right_y].y() << endl;
        if(!(isinf(dup_grid.grid[right_x][right_y].x()) || isinf(dup_grid.grid[right_x][right_y].y())) && abs(boost::geometry::distance(dup_grid.grid[right_x][right_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[right_x][right_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[right_x][right_y],en.boost_pt);
                best.first = Point(dup_grid.grid[right_x][right_y]);
                best.second = {right_x,right_y};
            }
            else
            {
                dup_grid.grid[right_x][right_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[right_x][right_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else
        {
            // cout << "obs\n";
        }
    }
}

void Planner::right_down(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int right_down_x = follow_path.back().second.first + 1;
    int right_down_y = follow_path.back().second.second - 1;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "right_down_x: " << right_down_x << ", " << " right_down_y: " << right_down_y << endl;
        
    if (0 <= right_down_x && right_down_x <= max_border_x && 0 <= right_down_y && right_down_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[right_down_x][right_down_y].x() << ", y: " << dup_grid.grid[right_down_x][right_down_y].y() << endl;
        if(!(isinf(dup_grid.grid[right_down_x][right_down_y].x()) || isinf(dup_grid.grid[right_down_x][right_down_y].y())) && abs(boost::geometry::distance(dup_grid.grid[right_down_x][right_down_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[right_down_x][right_down_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[right_down_x][right_down_y],en.boost_pt);
                best.first = Point(dup_grid.grid[right_down_x][right_down_y]);
                best.second = {right_down_x,right_down_y};
            }
            else
            {
                dup_grid.grid[right_down_x][right_down_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[right_down_x][right_down_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else
        {
            // cout << "obs\n";
        }
    }
}

void Planner::down(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int down_x = follow_path.back().second.first;
    int down_y = follow_path.back().second.second - 1;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "down_x: " << down_x << ", " << " down_y: " << down_y << endl;
        
    if (0 <= down_x && down_x <= max_border_x && 0 <= down_y && down_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[down_x][down_y].x() << ", y: " << dup_grid.grid[down_x][down_y].y() << endl;
        if(!(isinf(dup_grid.grid[down_x][down_y].x()) || isinf(dup_grid.grid[down_x][down_y].y())) && abs(boost::geometry::distance(dup_grid.grid[down_x][down_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[down_x][down_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[down_x][down_y],en.boost_pt);
                best.first = Point(dup_grid.grid[down_x][down_y]);
                best.second = {down_x,down_y};
            }
            else
            {
                dup_grid.grid[down_x][down_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[down_x][down_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else
        {
            // cout << "obs\n";
        }
    }
}

void Planner::down_left(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int down_left_x = follow_path.back().second.first - 1;
    int down_left_y = follow_path.back().second.second - 1;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "down_left_x: " << down_left_x << ", " << " down_left_y: " << down_left_y << endl;
        
    if (0 <= down_left_x && down_left_x <= max_border_x && 0 <= down_left_y && down_left_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[down_left_x][down_left_y].x() << ", y: " << dup_grid.grid[down_left_x][down_left_y].y() << endl;
        if(!(isinf(dup_grid.grid[down_left_x][down_left_y].x()) || isinf(dup_grid.grid[down_left_x][down_left_y].y())) && abs(boost::geometry::distance(dup_grid.grid[down_left_x][down_left_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[down_left_x][down_left_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[down_left_x][down_left_y],en.boost_pt);
                best.first = Point(dup_grid.grid[down_left_x][down_left_y]);
                best.second = {down_left_x,down_left_y};
            }
            else
            {
                dup_grid.grid[down_left_x][down_left_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[down_left_x][down_left_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else
        {
            // cout << "obs\n";
        }
    }
}

void Planner::left(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int left_x = follow_path.back().second.first - 1;
    int left_y = follow_path.back().second.second;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "left_x: " << left_x << ", " << " left_y: " << left_y << endl;
    
    if (0 <= left_x && left_x <= max_border_x && 0 <= left_y && left_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[left_x][left_y].x() << ", y: " << dup_grid.grid[left_x][left_y].y() << endl;
        if(!(isinf(dup_grid.grid[left_x][left_y].x()) || isinf(dup_grid.grid[left_x][left_y].y())) && abs(boost::geometry::distance(dup_grid.grid[left_x][left_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[left_x][left_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[left_x][left_y],en.boost_pt);
                best.first = Point(dup_grid.grid[left_x][left_y]);
                best.second = {left_x,left_y};
            }
            else
            {
                dup_grid.grid[left_x][left_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[left_x][left_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else{
            // cout << "obs\n";
        }
    }
}

void Planner::left_up(Point en, pair<Point,pair<int,int>> &best,float &best_dist)
{
    int left_up_x = follow_path.back().second.first - 1;
    int left_up_y = follow_path.back().second.second + 1;
    boost_point current_pt = follow_path[follow_path.size()-1].first.boost_pt;
    // cout << "current pt x :" << current_pt.x() << ", y: " << current_pt.y() << endl;
    // cout << "left_up_x: " << left_up_x << ", " << " left_up_y: " << left_up_y << endl;
        
    if (0 <= left_up_x && left_up_x <= max_border_x && 0 <= left_up_y && left_up_y <= max_border_y)
    {
        // cout << "x: " << dup_grid.grid[left_up_x][left_up_y].x() << ", y: " << dup_grid.grid[left_up_x][left_up_y].y() << endl;
        if(!(isinf(dup_grid.grid[left_up_x][left_up_y].x()) || isinf(dup_grid.grid[left_up_x][left_up_y].y())) && abs(boost::geometry::distance(dup_grid.grid[left_up_x][left_up_y],current_pt)) < 5)
        {
            if(boost::geometry::distance(dup_grid.grid[left_up_x][left_up_y],en.boost_pt) <= best_dist)
            {
                dup_grid.grid[best.second.first][best.second.second].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[best.second.first][best.second.second].y(std::numeric_limits<float>::infinity());
                best_dist = boost::geometry::distance(dup_grid.grid[left_up_x][left_up_y],en.boost_pt);
                best.first = Point(dup_grid.grid[left_up_x][left_up_y]);
                best.second = {left_up_x,left_up_y};
            }
            else
            {
                dup_grid.grid[left_up_x][left_up_y].x(std::numeric_limits<float>::infinity());
                dup_grid.grid[left_up_x][left_up_y].y(std::numeric_limits<float>::infinity());
            }
            // cout << "not a obs\n";
        }
        else
        {
            // cout << "obs\n";
        }
    }
}

void Planner::get_waypoints()
{
    if(this->follow_path.size() > 0)
    {
        for(size_t i = 0; i < this->follow_path.size(); i++)
        {
            if(i == 0)
            {
                Pose2d p;
                p.x = start.position.x;
                p.y = start.position.y;

                tf2::Quaternion q(
                    start.orientation.x,
                    start.orientation.y,
                    start.orientation.z,
                    start.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                p.th = yaw;

                this->waypoints.push_back(p);
            }

            else if(i == this->waypoints.size() - 1)
            {
                Pose2d p;
                p.x = end.position.x;
                p.y = end.position.y;

                tf2::Quaternion q(
                    end.orientation.x,
                    end.orientation.y,
                    end.orientation.z,
                    end.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                p.th = yaw;

                this->waypoints.push_back(p);
            }
            else
            {
                Pose2d p;
                p.x = follow_path[i].first.ros_pose.position.x;
                p.y = follow_path[i].first.ros_pose.position.y;

                tf2::Quaternion q(
                    follow_path[i].first.ros_pose.orientation.x,
                    follow_path[i].first.ros_pose.orientation.y,
                    follow_path[i].first.ros_pose.orientation.z,
                    follow_path[i].first.ros_pose.orientation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                p.th = yaw;

                this->waypoints.push_back(p);
            }
        }
    }
}

vector<Dubins_arc> Planner::multipoints()
{
    this->best_path.insert(this->best_path.begin(),this->waypoints.begin(),this->waypoints.end());

    float overall_length = 0.0;

	for
	(size_t i = this->best_path.size() - 2; i > 0; i--)
	{
		float best_path_length = std::numeric_limits<float>::max();
		int best_path_index;

		for 
		(int k = 0; k < step; k++)
		{
			Dubins_curve temp_curve;
			float temp_ang = k * angle_step;
			temp_curve = dubins_shortest_path(best_path[i].x, best_path[i].y, temp_ang, best_path[i+1].x, best_path[i+1].y, best_path[i+1].th, 5.0);

			if ((temp_curve.L + overall_length) < (best_path_length + overall_length))
			{
				best_path_length = temp_curve.L;
				best_path_index = k;
			}
		}
		best_path[i].th = best_path_index * angle_step;
		overall_length = overall_length + best_path_length;
	}

    cout << "============Waypoints Start============\n";
    for(size_t i = 0; i < this->best_path.size();i++)
    {
        cout << "i: " << i << " x: " << this->best_path[i].x << " y: " << this->best_path[i].y << " y: " << this->best_path[i].th << endl;
    }
    cout << "============Waypoints End=============\n";

    for 
	(size_t i = 0; i < best_path.size()-1; i++)
	{
		Dubins_curve temp;
        temp = dubins_shortest_path(best_path[i].x, best_path[i].y, best_path[i].th, best_path[i+1].x, best_path[i+1].y, best_path[i+1].th, 5.0);

		this->arcs.push_back(temp.a1);
		this->arcs.push_back(temp.a2);
		this->arcs.push_back(temp.a3);
	}

    return this->arcs;
}

vector<Dubins_arc> Planner::plan()
{
    vector<Dubins_arc> ret;
    find_path();
	get_waypoints();
    ret = multipoints();

    return ret;
}

void Planner::print_waypoints()
{   
    cout << "============Waypoints Start============\n";
    for 
    (size_t i = 0; i < this->waypoints.size(); i++)
    {
        cout << "i: " << i << " x: " << this->waypoints[i].x << " y: " << this->waypoints[i].y << " y: " << this->waypoints[i].th << endl;
    }
    cout << "============Waypoints End=============\n";
}

void Planner::print_followpath()
{   
    cout << "============FollowPath Start============\n";
    for 
    (size_t i = 0; i < follow_path.size(); i++)
    {
        cout << "i: " << i << " x: " << follow_path[i].first.boost_pt.x() << " y: " << follow_path[i].first.boost_pt.y() << endl;
    }
    cout << "============FollowPath End=============\n";
}

void Planner::print()
{
    cout << "start x: " << start.position.x << " start y: " << start.position.y << endl;
    cout << "end x: " << end.position.x << " start y: " << end.position.y << endl;
}

float round_up
(float value, int decimal_places)
{
	const float multiplier = std::pow(10.0, decimal_places);
	return std::ceil(value * multiplier) / multiplier;
}

float sinc(float t){
    float s;
    if(abs(t)<0.002){
        s = 1 - pow(t,(2/6)) * (1 - pow(t,2/20));
        
    }else{
        s = sin(t)/t;
    }
    return s;
}

float mod2pi(float ang){
    float out = ang;
    while(out < 0){
        out = out + 2 * M_PI;
    }
    while(out >= 2 * M_PI){
        out = out - 2 * M_PI;
    }
    return out;
}
