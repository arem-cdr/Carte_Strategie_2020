#pragma once

#include <mbed.h>
#include <iostream>
#include <vector>

#include "RRT.h"
#include "Point.h"
#include "Obstacle.h"
#include "Vector2f.h"

class PathfindingDebug // à remplacer par le classe gérant la strat plus tard
{
protected:
    RRT MyRRT;
    Point pos;
    Point target;
    std::vector<Obstacle> obstacles;
    float time;

public:
    PathfindingDebug(Point point = Point(Vector2f(150.f, 100.f), 0.f));
    ~PathfindingDebug() {}

    void update(float time);

    void Qtdisplay(Serial *pc);
    void debugDisplay();
};