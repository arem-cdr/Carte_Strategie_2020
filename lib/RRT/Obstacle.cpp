#include "Obstacle.h"

Obstacle::Obstacle(Vector2i pos, int radius) : Object(pos, radius), speed()
{
}

Obstacle::~Obstacle()
{
}

void Obstacle::updatePos(const float &dt)
{
    pos = pos + speed * dt;
}

void Obstacle::Qtdisplay(Serial *pc) const
{
    switch (this->radius)
    {
    case gobeletRadius:
        pc->printf("%d\n%d\n%d\n", 2, this->getPos().getX(), this->getPos().getY());
        break;
    case robotRadius:
        pc->printf("%d\n%d\n%d\n", 3, this->getPos().getX(), this->getPos().getY());
        break;
    default:
        break;
    }
}