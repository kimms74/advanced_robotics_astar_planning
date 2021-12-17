#include <iostream>
#include "Astar.h"

int main()
{
    int scale = 5;
    int grid_multiplier = 1;    //make grid size up
    const int height = 20 * scale;
    const int width = 40 * scale;
    

    //To do: change input position to real input position
    double safety_value = 2;
    AStar::Obs2i obs1{ 5 * scale,10 * scale,8 * scale }; //real position & size (cm)
    AStar::Obs2i obs2{ 17 * scale,20 * scale,5 * scale };
    AStar::Obs2i obs3{ 10 * scale,32 * scale,2 * scale };

    AStar::Vec2i start{ 1,1 };
    AStar::Vec2i target1{ height - 2,width - 2 };
    AStar::Vec2i target2{ 1, width - 2 };
    AStar::Vec2i target3{ height - 2,1 };

    AStar::Generator generator;
    generator.setWorldSize({ height, width }, grid_multiplier);
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    generator.addCollision(obs1, obs2, obs3, safety_value);

    std::cout << "Generate path ... \n";
    AStar::CoordinateList path;
    //To do: make real input position to grid position?
    generator.findPath(path, target2, target3);
    generator.findPath(path, target1, target2);
    generator.findPath(path, start, target1);

    std::reverse(path.begin(), path.end());

    //for (auto& coordinate : path) {
    //    std::cout << coordinate.x << " " << coordinate.y << "\n";
    //}

    generator.printMap();


    AStar::CoordinateList robot_path;
    std::cout << path[0].x << " " << path[0].y << " " << path.size() << "\n";
    //std::cout << path1[0].x << " " << path1[0].y << " " << path1.size() << "\n";

    std::cout << path[198].x << " " << path[198].y << " " << path.size() << "\n";
    //std::cout << path2[0].x << " " << path2[0].y << " " << path2.size() << "\n";

    std::cout << path[296].x << " " << path[296].y << " " << path.size() << "\n";
    //std::cout << path3[0].x << " " << path3[0].y << " " << path3.size() << "\n";

    //To do: write code changeRealPath
    generator.changeRealPath(path, robot_path);
}