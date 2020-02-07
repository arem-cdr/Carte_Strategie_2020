#include "Obstacle.h"

Obstacle::Obstacle(Vector2f pos, object_type objectType) : Object(pos), speed()
{
    if (objectType == GOBELET)
        radius = gobeletRadius;
    else if (objectType == ROBOT)
        radius = robotRadius;
}

Obstacle::~Obstacle()
{
}

void Obstacle::updatePos(const float &dt)
{
    pos.x = pos.x + speed.x * dt;
    pos.y = pos.y + speed.y * dt;

    //if (dist(speed) < 3.f)
    //    speed = Vector2f(0.f, 0.f);
}