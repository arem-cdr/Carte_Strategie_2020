#pragma once

#include <mbed.h>

#include "Vector2i.h"
#include "Object.h"
#include "RRTConstantes.h"

class Obstacle : public Object
{
protected:
    Vector2i speed;

public:
    Obstacle(Vector2i pos = Vector2i(0.f, 0.f), int radius = gobeletRadius);
    ~Obstacle();

    void updatePos(const float &dt); //update object pos according to current speed

    virtual void Qtdisplay(Serial *pc) const = 0;
};
