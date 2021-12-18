#include <iostream>
#include "../include/Astar.h"

int fact(int n) {
    return (n == 0) || (n == 1) ? 1 : n * fact(n - 1);
}

double binomial(int n, int k) {
    return fact(n) / (fact(k) - fact(n - k));
}


struct Position
{
    double x;
    double y;
};

int main()
{
    int scale = 2;
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

    generator.Simplepath(path, obs1, obs2, obs3);


   

   /* for (auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }*/

    generator.printMap();

    return 0;

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

    
  /*  for (int i = 0; i < robot_path.size(); i++) {
        std::cout << robot_path[i].x << robot_path[i].y << std::endl;
    }*/

    //std::cout << fact(3);

    //double b_pos_x[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 }; //bezire position x
    //double b_pos_y[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };//bezire position y
    //std::cout << b_pos_x[0] << std::endl;

    /*std::vector<Position> point;
    Position p1, p2, p3;
    p1.x = 1;
    p1.y = 2;
    p2.x = 3;
    p2.y = 4;
    p3.x = 5;
    p3.y = 6;*/
    //std::cout << p1.x << std::endl;
    /*point.push_back(p1);
    point.push_back(p2);
    point.push_back(p3);*/
    //std::cout << point.size() << std::endl;

    //point.erase(point.begin() + 1);
    //std::cout << point.size() << std::endl;
    //std::cout << point[1].x << point[1].y << std::endl;

   /* for (int i = 0; i < point.size(); i++) {
        std::cout << "iter" << i << std::endl;
        std::cout << point[i].x << point[i].y << std::endl;
        point.erase(point.begin() + i);
    }*/

    //std::cout << "before erase : " << robot_path.size() << std::endl;
    //double r1, r2, r3;
    //r1 = obs1.d / 2;
    //r2 = obs2.d / 2;
    //r3 = obs3.d / 2;

    //double a, b, c;
    //double r_x, r_y, r_0;
    //double x_i, y_i, x_m, y_m, x_f, y_f;
    //double d_m, d_f;

    //std::cout << distance(1, 2, 3, 4);

    //for (int i = 0; i < robot_path.size(); i++) {
    //    r_x = obs1.x;
    //    r_y = obs1.y;
    //    r_0 = r1;
    //    x_i = robot_path[i].x;
    //    y_i = robot_path[i].y;

    //    // nearest obstacle select
    //    if (distance(r_x, r_y, x_i, y_i) > distance(obs2.x, obs2.y, x_i, y_i)) {
    //        r_x = obs2.x;
    //        r_y = obs2.y;
    //        r_0 = r2;
    //    }
    //    else if(distance(r_x, r_y, x_i, y_i) > distance(obs3.x, obs3.y, x_i, y_i)){
    //        r_x = obs3.x;
    //        r_y = obs3.y;
    //        r_0 = r3;
    //    }
    //    
    //    // collision check of middle point
    //    x_m = (robot_path[i].x + robot_path[i + 1].x) / 2;
    //    y_m = (robot_path[i].y + robot_path[i + 1].y) / 2;
    //    d_m = distance(r_x, r_y, x_m, y_m);
    //    if (d_m >= r_0) {
    //        x_f = robot_path[i + 1].x;
    //        y_f = robot_path[i + 1].y;
    //        d_f = distance(r_x, r_y, x_f, y_f);
    //        if (d_f >= r_0) {
    //            robot_path.erase(robot_path.begin() + i);
    //        }
    //    }
    //}

    //std::cout << "after erase : " << robot_path.size() << std::endl;



}