#include <iostream>
#include "Astar.h"

int main()
{
    int grid_multiplier = 1;    //make grid size up
    const int height = 20;
    const int width = 40;

    //To do: change input position to real input position
    double safety_value = 2;
    AStar::Obs2i obs1{ 3,10,8 }; //real position & size (cm)
    AStar::Obs2i obs2{ 17,20,5 };
    AStar::Obs2i obs3{ 10,35,5 };

    AStar::Vec2i start{ 1,1 };
    AStar::Vec2i target1{ 18,38 };
    AStar::Vec2i target2{ 1, 38 };
    AStar::Vec2i target3{ 18,1 };

    AStar::Generator generator;
    generator.setWorldSize({ height, width }, grid_multiplier);
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    generator.addCollision(obs1, obs2, obs3, safety_value);

    std::cout << "Generate path ... \n";
    AStar::CoordinateList path;
    //To do: make real input position to grid position?
    generator.findPath(path, start, target1);
    generator.findPath(path, target1, target2);
    generator.findPath(path, target2, target3);

    //for (auto& coordinate : path) {
    //    std::cout << coordinate.x << " " << coordinate.y << "\n";
    //}

    generator.printMap();

    AStar::CoordinateList robot_path;
    //To do: write code changeRealPath
    generator.changeRealPath(path, robot_path);
}