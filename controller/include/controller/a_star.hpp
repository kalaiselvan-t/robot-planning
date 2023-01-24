#pragma once

#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/algorithms/within.hpp> 
#include <boost/range/adaptor/indexed.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include <boost/assign/list_of.hpp>

class AStarPlanner
{
private:
    class Node
    {
    public:
        Node(const boost::geometry::model::d2::point_xy<float>& position)
            : position_(position),
              occupied_(false),
              parent(NULL),
              g(0),
              h(0) {};

        ~Node() {};

        void SetOccupied(bool state)
        {
            this->occupied_ = state;
        }

        float GetTotalCost()
        {
            return g + h;
        }

        void print()
        {
            std::cout << "x: " << position_.x() << " y: " << position_.y() << " occupied: " << occupied_ << std::endl;
        }

        bool operator==(Node* rhs)
        {
            return this->position_.x() == rhs->position_.x() && this->position_.y() == rhs->position_.y();
        }

        boost::geometry::model::d2::point_xy<float> position_;
        Node* parent;
        bool occupied_;
        float g;
        float h; // heuristic
    };

    std::vector<Node> gridMap;
    float gridResolution;
    int gridWidth;
    int gridHeight;
    boost::geometry::model::d2::point_xy<float> gridOrigin;
    boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float>> boundaries;

    const std::array<boost::geometry::model::d2::point_xy<float>, 8> adjacentCoordinates = {
        boost::geometry::model::d2::point_xy<float>(0.0, -1.0),
        boost::geometry::model::d2::point_xy<float>(0.0, 1.0),
        boost::geometry::model::d2::point_xy<float>(-1.0, 0.0),
        boost::geometry::model::d2::point_xy<float>(1.0, 0.0),
        boost::geometry::model::d2::point_xy<float>(-1.0, -1.0),
        boost::geometry::model::d2::point_xy<float>(-1.0, 1.0),
        boost::geometry::model::d2::point_xy<float>(1.0, -1.0),
        boost::geometry::model::d2::point_xy<float>(1.0, 1.0)
    };

public:
    AStarPlanner();
    ~AStarPlanner();

    void CreateGridMap(
        boost::geometry::model::d2::point_xy<float>& boundLowerLeft,
        boost::geometry::model::d2::point_xy<float>& boundUpperLeft,
        boost::geometry::model::d2::point_xy<float>& boundUpperRight,
        boost::geometry::model::d2::point_xy<float>& boundLowerRight,
        float gridMapResolution);
    void PrintGridMap();
    void PrintGridState(int idx);
    std::vector<boost::geometry::model::d2::point_xy<float>> FindPath(
        const boost::geometry::model::d2::point_xy<float>& start,
        const boost::geometry::model::d2::point_xy<float>& end);
    int GetGridMapNodeIndex(const boost::geometry::model::d2::point_xy<float>& position);
};
