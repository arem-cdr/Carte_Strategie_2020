#pragma once

#include "Vector2f.h"

class Object
{
protected:
    float radius;
    Vector2f pos;

public:
    Object(Vector2f pos = Vector2f(0.f, 0.f), float r = 0.f)
    {
        this->pos = pos;
        this->radius = r;
    }
    ~Object() {}

    float getRadius() const { return radius; }
    Vector2f getPos() const { return pos; }
};
