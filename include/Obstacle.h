#pragma once

#include "Vector2f.h"
#include "BasicFunctions.h"
#include "Object.h"
#include "Constantes.h"

enum object_type
{
    DEFAULT = 0,
    GOBELET,
    ROBOT
};

class Obstacle : public Object
{
private:
    Vector2f speed;

public:
    Obstacle(Vector2f pos = Vector2f(0.f, 0.f), object_type objectType = GOBELET);
    ~Obstacle();

    void updatePos(const float &dt); //update object pos according to current speed
};
