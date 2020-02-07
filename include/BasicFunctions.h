#pragma once

#include <vector>

#include "Vector2f.h"

template <typename T>
T min(T a, T b)
{
    if (a < b)
        return a;
    return b;
}

template <typename T>
T max(T a, T b)
{
    if (a > b)
        return a;
    return b;
}

bool segmentIntersectCircle(const Vector2f &p0, const Vector2f &p1, const Vector2f &c, const float &r);

float dist(std::vector<Vector2f> vector);