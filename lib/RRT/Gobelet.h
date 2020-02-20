#pragma once

#include <mbed.h>

#include "Vector2i.h"
#include "Obstacle.h"
#include "RRTConstantes.h"

enum Color
{
    DEFAULT = 0,
    GREEN,
    RED
};

class Gobelet : public Obstacle
{
private:
    Color color;

public:
    Gobelet(Vector2i pos = Vector2i(0, 0), Color color = GREEN) : Obstacle(pos, gobeletRadius), color(color) {}
    ~Gobelet() {}

    virtual void Qtdisplay(Serial *pc) const { pc->printf("%d\n%d\n%d\n", color, this->getPos().getX(), this->getPos().getY()); }
};
