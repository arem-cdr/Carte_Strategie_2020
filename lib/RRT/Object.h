#pragma once

#include "Vector2f.h"

class Object
{
protected:
    int radius;
    Vector2i pos;

public:
    Object(Vector2i pos = Vector2i(0.f, 0.f), int r = 0.f)
    {
        this->pos = pos;
        this->radius = r;
    }
    ~Object() {}

    int getRadius() const { return radius; }
    Vector2i getPos() const { return pos; }
};
