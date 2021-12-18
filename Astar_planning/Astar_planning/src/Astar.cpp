#include "Astar.h"

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

bool AStar::Vec2f::operator == (const Vec2f& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node* parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 },
        { -1, -2 }, { 1, 2 }, { -1, 2 }, { 1, -2 }
        
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_, int grid_multiplier_)
{
    worldSize.x = worldSize_.x * grid_multiplier_;
    worldSize.y = worldSize_.y * grid_multiplier_;
    grid_multiplier = grid_multiplier_;

    grid_map.resize(worldSize.x, std::vector<std::string>(worldSize.y, "0"));
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 12 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(AStar::Obs2i obs1_, AStar::Obs2i obs2_, AStar::Obs2i obs3_, double safety_value_)
{
    for (int i = 0; i < worldSize.x; ++i)
    {
        for (int j = 0; j < worldSize.y; ++j)
        {
            int distance_1 = sqrt(pow(i / double(grid_multiplier) - obs1_.x, 2) + pow(j / double(grid_multiplier) - obs1_.y, 2));
            int distance_2 = sqrt(pow(i / double(grid_multiplier) - obs2_.x, 2) + pow(j / double(grid_multiplier) - obs2_.y, 2));
            int distance_3 = sqrt(pow(i / double(grid_multiplier) - obs3_.x, 2) + pow(j / double(grid_multiplier) - obs3_.y, 2));
            if (distance_1 <= (obs1_.d / 2 + safety_value_) || distance_2 <= (obs2_.d / 2 + safety_value_) || distance_3 <= (obs3_.d / 2 + safety_value_))
            {
                walls.push_back({ i,j });
                grid_map[i][j] = "1";
            }
        }
    }
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

void AStar::Generator::printMap()
{
    for (int i = 0; i < worldSize.x; ++i)
    {
        for (int j = 0; j < worldSize.y; ++j)
        {
            std::cout << grid_map[i][j] << " ";
            if (j == (worldSize.y - 1))
            {
                std::cout << std::endl;
            }
        }
    }
}

void AStar::Generator::findPath(CoordinateList& path_, Vec2i source_, Vec2i target_)
{
    //Vec2i start{ source_.x * grid_multiplier - 1, source_.y * grid_multiplier - 1 }; //change!!
    Vec2i start{ source_.x * grid_multiplier, source_.y * grid_multiplier }; //change!!
    Vec2i finish{ target_.x * grid_multiplier, target_.y * grid_multiplier };
    grid_map[start.x][start.y] = "*";

    Node* current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(500);
    closedSet.reserve(500);
    openSet.push_back(new Node(start));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == finish) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node* successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    //CoordinateList path;
    while (current != nullptr) {
        path_.push_back(current->coordinates);
        grid_map[current->coordinates.x][current->coordinates.y] = "*"; 
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);
}

void AStar::Generator::changeRealPath(const CoordinateList& path_, CoordinateListf& robot_path_)
{
    for (int i = 0; i < path_.size(); i++) {
        Vec2f new_path_;
        new_path_.x = -0.001 * path_[i].x + 0.1;
        new_path_.y = 0.001 * path_[i].y - 0.2;
        robot_path_.push_back(new_path_);
    }
    //To do: convert grid path to real robot path
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete* it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
