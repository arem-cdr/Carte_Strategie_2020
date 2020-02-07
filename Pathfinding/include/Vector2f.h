#pragma once

#include <iostream>
#include <cmath>

class Vector2f
{
public:
    Vector2f(float x = 0.f, float y = 0.f) : x(x), y(y) {}

    float getX() const { return x; }
    float getY() const { return y; }

private:
    float x;
    float y;

    // operateur d'affichage
    friend std::ostream &operator<<(std::ostream &flux, const Vector2f &v)
    {
        std::cout << v.x << " " << v.y << std::endl;
        return flux;
    }

    // operateurs de comparaison
    friend bool operator==(const Vector2f &v1, const Vector2f &v2) { return (v1.getX() == v2.getX()) && (v1.y == v2.y); }
    friend bool operator!=(const Vector2f &v1, const Vector2f &v2) { return !(v1 == v2); }

    // operateurs arithmetiques
    friend Vector2f operator+(const Vector2f &v1, const Vector2f &v2) { return Vector2f(v1.x + v2.x, v1.y + v2.y); }
    friend Vector2f operator-(const Vector2f &v1, const Vector2f &v2) { return Vector2f(v1.x - v2.x, v1.y - v2.y); }
    friend Vector2f operator*(const float &a, const Vector2f &v) { return Vector2f(a * v.x, a * v.y); }
    friend Vector2f operator*(const Vector2f &v, const float &a) { return Vector2f(a * v.x, a * v.y); }

    // operateurs d'assignement
    friend void operator+=(Vector2f &v1, const Vector2f &v2)
    {
        v1.x += v2.x;
        v1.y += v2.y;
    }
    friend void operator-=(Vector2f &v1, const Vector2f &v2)
    {
        v1.x -= v2.x;
        v1.y -= v2.y;
    }
    friend void operator*=(Vector2f &v, const float &a)
    {
        v.x *= a;
        v.y *= a;
    }

    friend float dot(const Vector2f &v1, const Vector2f &v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    friend float dist(const Vector2f &v)
    {
        return sqrt(v.x * v.x + v.y * v.y);
    }

    // classes amies
    friend class Obstacle;
};