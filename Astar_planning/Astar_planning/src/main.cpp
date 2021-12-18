#include <iostream>
#include "Astar.h"

int main()
{
    int scale = 10;
    int grid_multiplier = 1;    //make grid size up
    const int height = 20 * scale;
    const int width = 40 * scale;
    

    //To do: change input position to real input position
    double safety_value = 2;
    AStar::Obs2i obs1{ 5 * scale,10 * scale,8 * scale }; //real position & size (cm)
    AStar::Obs2i obs2{ 17 * scale,20 * scale,5 * scale };
    AStar::Obs2i obs3{ 10 * scale,32 * scale,2 * scale };

    AStar::Vec2i start{ 2 * scale,2 * scale };
    AStar::Vec2i target1{ height - 2 * scale - 1,width - 2 * scale - 1 };
    AStar::Vec2i target2{ 2 * scale, width - 2 * scale - 1 };
    AStar::Vec2i target3{ height - 2 * scale - 1,2 * scale };

    AStar::Generator generator;
    generator.setWorldSize({ height, width }, grid_multiplier);
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    generator.addCollision(obs1, obs2, obs3, safety_value);

    std::cout << "Generate path ... \n";
    AStar::CoordinateList path;
    //To do: make real input position to grid position?
    generator.findPath(path, target2, target3);
    int path_size3 = path.size();
    generator.findPath(path, target1, target2);
    int path_size2 = path.size() - path_size3;
    generator.findPath(path, start, target1);
    int path_size1 = path.size() - path_size2 - path_size3;

    std::reverse(path.begin(), path.end());

    //for (auto& coordinate : path) {
    //    std::cout << coordinate.x << " " << coordinate.y << "\n";
    //}

    generator.printMap();


    AStar::CoordinateListf robot_path;
    std::cout << path[0].x << " " << path[0].y << " " << path_size1 << "\n";
    std::cout << path[1].x << " " << path[1].y << " " << path_size1 << "\n";
    //std::cout << path1[0].x << " " << path1[0].y << " " << path1.size() << "\n";

    std::cout << path[path_size1-1].x << " " << path[path_size1-1].y << " " << path_size2 << "\n";
    std::cout << path[path_size1].x << " " << path[path_size1].y << " " << path_size2 << "\n";
    std::cout << path[path_size1+1].x << " " << path[path_size1+1].y << " " << path_size2 << "\n";
    //std::cout << path2[0].x << " " << path2[0].y << " " << path2.size() << "\n";

    std::cout << path[path_size1 + path_size2+1].x << " " << path[path_size1 + path_size2+1].y << " " << path_size3 << "\n";
    //std::cout << path3[0].x << " " << path3[0].y << " " << path3.size() << "\n";

    //To do: write code changeRealPath
    generator.changeRealPath(path, robot_path);
    std::cout << robot_path[0].x << " " << robot_path[0].y << " " << path_size1 << "\n";
    std::cout << robot_path[path_size1].x << " " << robot_path[path_size1].y << " " << path_size2 << "\n";
    std::cout << robot_path[path_size1 + path_size2].x << " " << robot_path[path_size1 + path_size2].y << " " << robot_path.size() << "\n";

}