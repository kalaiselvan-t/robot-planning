#include <set>

#include "../include/controller/a_star.hpp"

AStarPlanner::AStarPlanner()
    : gridResolution(1.0),
      gridWidth(0),
      gridHeight(0),
      gridOrigin(0, 0) {};

AStarPlanner::~AStarPlanner() {};

void AStarPlanner::CreateGridMap(
    boost::geometry::model::d2::point_xy<float>& boundLowerLeft,
    boost::geometry::model::d2::point_xy<float>& boundUpperLeft,
    boost::geometry::model::d2::point_xy<float>& boundUpperRight,
    boost::geometry::model::d2::point_xy<float>& boundLowerRight,
    float gridMapResolution)
{
    this->gridMap.clear();

    this->boundaries.clear();
    // boost::geometry::assign_points(this->boundaries, boost::assign::tuple_list_of(
    //     (boundLowerLeft.x(), boundLowerLeft.y()),
    //     (boundUpperLeft.x(), boundUpperLeft.y()),
    //     (boundUpperRight.x(), boundUpperRight.y()),
    //     (boundLowerRight.x(), boundLowerRight.y()),
    //     (boundLowerLeft.x(), boundLowerLeft.y()) // important to close loop for polygon
    // ));

    boundLowerLeft = boost::geometry::model::d2::point_xy<float>(-101, -101);
    boundUpperLeft = boost::geometry::model::d2::point_xy<float>(-101, 101);
    boundUpperRight = boost::geometry::model::d2::point_xy<float>(101, 101);
    boundLowerRight = boost::geometry::model::d2::point_xy<float>(101, -101);

    boost::geometry::model::box<boost::geometry::model::d2::point_xy<float>> box(boundLowerLeft, boundUpperRight);
    boost::geometry::assign(this->boundaries, box);

    bool v = boost::geometry::is_valid(this->boundaries);
    std::cout << "BOUNDARIES POLYGON VALID " << v << std::endl;

    boost::geometry::model::d2::point_xy<float> gridOrigin;
    gridOrigin = boundLowerLeft;

    float gridWidth = std::abs(gridOrigin.x() - boundLowerRight.x());
    float gridHeight = std::abs(gridOrigin.y() - boundUpperLeft.y());

    int stepsX = gridWidth / gridMapResolution + 1;
    int stepsY = gridHeight / gridMapResolution + 1;
    std::cout << "stepsX: " << stepsX << std::endl;
    std::cout << "stepsY: " << stepsY << std::endl;

    for (int y = 0; y < stepsY; y++)
    {
        float y_ = gridOrigin.y() + (y * gridMapResolution);
        // std::vector<Node> row;
        for (int x = 0; x < stepsX; x++)
        {
            float x_ = gridOrigin.x() + (x * gridMapResolution);
            boost::geometry::model::d2::point_xy<float> position(x_, y_);
            Node node(position);
            // TODO consider obstacles during grid creation?
            if (x_ == 0.0 && y_ == 0.0)
            {
                node.occupied_ = true;
            }
            // if (x_ == 3.0 && y_ == 0.0)
            // {
            //     node.occupied_ = true;
            // }
            this->gridMap.push_back(node);
        }
        // this->gridMap.push_back(row);
    }

    this->gridResolution = gridMapResolution;
    this->gridOrigin = gridOrigin;
    this->gridWidth = stepsX;
    this->gridHeight = stepsY;
}

void AStarPlanner::PrintGridMap()
{
    std::cout << "GridMap" << std::endl;
    std::cout << "gridWidth: " << this->gridWidth << std::endl;
    std::cout << "gridHeight: " << this->gridHeight << std::endl;

    for (size_t i = 0; i < this->gridMap.size(); i++)
    {
		int idx = GetGridMapNodeIndex(this->gridMap[i].position_);
        std::cout << "LoopIdx: " << i << " Idx: " << idx << " ";
        this->gridMap[i].print();
        // for (size_t x = 0; x < this->gridMap[y].size(); x++)
        // {
        //     this->gridMap[y][x].print();
        // }
    }
}

void AStarPlanner::PrintGridState(int idx)
{
    for (size_t i = this->gridMap.size()-1; i >= 0; i--)
    {
        std::cout << "o";
        if (i % this->gridWidth == 0) {
            std::cout << std::endl;
        }
        // int y = i / cols;
        // int x = i % cols;
        // for (size_t x = 0)
        // std::cout << "LoopIdx: " << i << " Idx: " << idx << " ";
        // this->gridMap[i].print();
        // for (size_t x = 0; x < this->gridMap[y].size(); x++)
        // {
        //     this->gridMap[y][x].print();
        // }
    }
    std::cout << " " << std::endl;
    std::cout << " " << std::endl;
    std::cout << " " << std::endl;
}

std::vector<boost::geometry::model::d2::point_xy<float>> AStarPlanner::FindPath(
    const boost::geometry::model::d2::point_xy<float>& start,
    const boost::geometry::model::d2::point_xy<float>& end)
{
    std::vector<boost::geometry::model::d2::point_xy<float>> path;

    // create start and end node
    // float roundedStart = std::round_to_nearest()
    // float precision = this->gridResolution;
    // int decimals = std::pow(10, precision);
    // float sX = (std::round(start.x() * decimals)) / decimals;
    // float sY = (std::round(start.y() * decimals)) / decimals;
    // float eX = (std::round(end.x() * decimals)) / decimals;
    // float eY = (std::round(end.y() * decimals)) / decimals;

    int gridIndexStart = GetGridMapNodeIndex(start);
    int gridIndexEnd = GetGridMapNodeIndex(end);
    // int gridIndexStart = GetGridMapNodeIndex(boost::geometry::model::d2::point_xy<float>(sX, sY));
    // int gridIndexEnd = GetGridMapNodeIndex(boost::geometry::model::d2::point_xy<float>(eX, eY));

    std::cout << "sx: " << start.x() << " sy: " << start.y() << " index: " << gridIndexStart << std::endl;
    std::cout << "sx: " << this->gridMap[gridIndexStart].position_.x() << " sy: " << this->gridMap[gridIndexStart].position_.y() << std::endl;

    std::cout << "ex: " << end.x() << " ey: " << end.y()  << " index: " << gridIndexEnd << std::endl;
    std::cout << "ex: " << this->gridMap[gridIndexEnd].position_.x() << " ey: " << this->gridMap[gridIndexEnd].position_.y() << std::endl;

    // return {};

    // TODO check grid bounds
    // if (startNode->occupied_)
    // {
    //     std::cout << "start location occupied" << std::endl;
    //     return path;
    // }
    // if (endNode->occupied_)
    // {
    //     std::cout << "end location occupied" << std::endl;
    //     return path;
    // }

    std::set<int> openList;
    std::set<int> closedList;
    openList.insert(gridIndexStart);

    int maxSizeOL = 0;
    int maxSizeCL = 0;
    while (!openList.empty())
    {
        if (maxSizeOL < openList.size()) {
            maxSizeOL = openList.size();
        }
        if (maxSizeCL < closedList.size()) {
            maxSizeCL = closedList.size();
        }

        // std::cout << "NEW_ITERATION-------------------------------------------------------------" << std::endl;
        // std::cout << "openListLength " << openList.size() << std::endl;
        // std::cout << "closedListLength " << closedList.size() << std::endl;
        int gridIndexCurrent = *openList.begin();

        // select cell with lowest total cost
        for (const auto& gridIndex : openList)
        {
            if (this->gridMap[gridIndex].GetTotalCost() < this->gridMap[gridIndexCurrent].GetTotalCost())
            {
                gridIndexCurrent = gridIndex;
            }
        }

        openList.erase(gridIndexCurrent);
        closedList.insert(gridIndexCurrent);

        if (gridIndexCurrent == gridIndexEnd)
        {
            // std::cout << "REACHED GOAL" << std::endl;
            // std::cout << "x: " << this->gridMap[gridIndexEnd].position_.x() << " y: " << this->gridMap[gridIndexEnd].position_.y() << std::endl;
            Node* n = &this->gridMap[gridIndexEnd];
            while (n->parent != NULL)
            {
                // std::cout << " --- " << std::endl;
                // n->print();
                // std::cout << "parent" <<std::endl;
                // n->parent->print();
                path.push_back(n->position_);
                n = n->parent;
            }
            path.push_back(n->position_); // push last node
            std::reverse(path.begin(), path.end());
            return path;
        }

        // std::cout << "SEARCHING" << std::endl;
        // std::cout << "GridCurrentIndex " << gridIndexCurrent << " from OPEN_LIST" << std::endl;

        // process adjacent nodes
        for (const auto& adj : this->adjacentCoordinates)
        {
            // get node position
            boost::geometry::model::d2::point_xy<float> nodePosition(
                (adj.x() * this->gridResolution) + this->gridMap[gridIndexCurrent].position_.x(),
                (adj.y() * this->gridResolution) + this->gridMap[gridIndexCurrent].position_.y()
            );
            // std::cout << " ADJ POS x: " << nodePosition.x() << " y: " << nodePosition.y() << std::endl;
            // make sure within defined grid and not occupied
            int gridIndexAdjacent = GetGridMapNodeIndex(nodePosition);
            if (boost::geometry::within(nodePosition, this->boundaries) &&
                this->gridMap[gridIndexAdjacent].occupied_ == false &&
                closedList.count(gridIndexAdjacent) == 0)
            {
                // calculate cost
                float g = this->gridMap[gridIndexCurrent].g + this->gridResolution;
                float h = std::pow(this->gridMap[gridIndexEnd].position_.x() - this->gridMap[gridIndexAdjacent].position_.x(), 2.0)
                        + std::pow(this->gridMap[gridIndexEnd].position_.y() - this->gridMap[gridIndexAdjacent].position_.y(), 2.0);

                // std::cout << "cost " << g << " currcost " << this->gridMap[gridIndexAdjacent].g << std::endl;
                if (this->gridMap[gridIndexAdjacent].g == 0 || g < this->gridMap[gridIndexAdjacent].g)
                {
                    // update due to lower cost
                    // std::cout << "UPDATE" << std::endl;
                    this->gridMap[gridIndexAdjacent].g = g;
                    this->gridMap[gridIndexAdjacent].h = h;
                    this->gridMap[gridIndexAdjacent].parent = &this->gridMap[gridIndexCurrent];
                }

                openList.insert(gridIndexAdjacent);
                // std::cout << "adj" << std::endl;
            }

            // if (gridIndexAdjacent == gridIndexEnd)
            // {
            //     std::cout << "REACHED GOAL" << std::endl;
            //     return {};
            // }
            // else
            // {
            //     std::cout << " GRID INDEX END " << gridIndexEnd << std::endl;
            //     std::cout << " GRID INDEX ADJ " << gridIndexAdjacent << std::endl;
            //     this->gridMap[gridIndexCurrent].print();
            // }
        }

        // std::vector<Node*> adjacentNodes;
        // for (const auto& adj : this->adjacentCoordinates)
        // {
        //     // get node position
        //     boost::geometry::model::d2::point_xy<float> nodePosition(
        //         (adj.x() * this->gridResolution) + currentNode->position_.x(),
        //         (adj.y() * this->gridResolution) + currentNode->position_.y()
        //     );

        //     // make sure within defined grid
        //     if (!boost::geometry::within(nodePosition, this->boundaries))
        //     {
        //         std::cout << "outside boundaries" << std::endl;
        //         std::cout << "boundaries: " << std::endl;
        //         for (auto& p : this->boundaries.outer())
        //         {
        //             std::cout << "x: " << p.x() << " y: " << p.y() << std::endl;
        //         }
        //         std::cout << "node " << std::endl;
        //         std::cout << "x: " << nodePosition.x() << " y: " << nodePosition.y() << std::endl;
        //         std::cout << "x: " << nodePosition.x() << " y: " << nodePosition.y() << std::endl;
        //         continue;
        //     }

        //     // make sure it is not occupied
        //     int nodeIndex = GetGridMapNodeIndex(nodePosition);
        //     Node* adjacentNode = &this->gridMap[nodeIndex];
        //     std::cout << "AdjacentNode " << &adjacentNode << " PARENT " << adjacentNode->parent << " from GRID_MAP" << std::endl;
        //     // Node adjacentNode(this->gridMap[nodeIndex]);
        //     if (adjacentNode->occupied_)
        //     {
        //         std::cout << "occupied space" << std::endl;
        //         continue;
        //     }


        //     adjacentNode->distanceToStartNode = g;
        //     adjacentNode->distanceEstimateToEndNode = h;
        //     adjacentNode->parent = currentNode;
        //     openList.push_back(adjacentNode);
    }

    std::cout << "EXIT / NO PATH FOUND" << std::endl;
    std::cout << "maxOLsize: " << maxSizeOL << std::endl;
    std::cout << "maxCLsize: " << maxSizeCL << std::endl;
    std::cout << "MAX_SIZE_OL: " << openList.max_size() << std::endl;
    std::cout << "MAX_SIZE_CL: " << closedList.max_size() << std::endl;
    return {};
}

int AStarPlanner::GetGridMapNodeIndex(const boost::geometry::model::d2::point_xy<float>& position)
{
    int yIndex = (position.y() - gridOrigin.y()) / this->gridResolution;
    int xIndex = (position.x() - gridOrigin.x()) / this->gridResolution;
    return yIndex * this->gridHeight + xIndex;
}
