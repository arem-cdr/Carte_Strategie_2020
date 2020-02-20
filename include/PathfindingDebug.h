#pragma once

#include <mbed.h>
#include <iostream>
#include <vector>

#include "RRT.h"
#include "Point.h"
#include "Obstacle.h"
#include "Gobelet.h"
#include "Vector2f.h"

class PathfindingDebug // à remplacer par le classe gérant la strat plus tard
{
protected:
    RRT MyRRT;
    Point pos;
    Point target;
    std::vector<Obstacle *> obstacles;

    float globalTime;
    //float last;

public:
    PathfindingDebug(Point point = Point(Vector2i(150, 100), 0.f));
    ~PathfindingDebug();

    void mainLoop(Serial *pc);

    void update();

    void Qtdisplay(Serial *pc);
    void debugDisplay();
};