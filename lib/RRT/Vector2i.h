#pragma once

#include <iostream>
#include <cmath>

class Vector2i
{
public:
    Vector2i(int x = 0.f, int y = 0.f) : x(x), y(y) {}

    int getX() const { return x; }
    int getY() const { return y; }

private:
    int x;
    int y;

    // operateur d'affichage

    friend std::ostream &operator<<(std::ostream &flux, const Vector2i &v)
    {
        std::cout << v.x << " " << v.y << std::endl;
        return flux;
    }

    // operateurs de comparaison

    friend bool operator==(const Vector2i &v1, const Vector2i &v2) { return (v1.x == v2.x) && (v1.y == v2.y); }
    friend bool operator!=(const Vector2i &v1, const Vector2i &v2) { return !(v1 == v2); }

    // operateurs arithmetiques

    friend Vector2i operator+(const Vector2i &v1, const Vector2i &v2) { return Vector2i(v1.x + v2.x, v1.y + v2.y); }
    friend Vector2i operator-(const Vector2i &v1, const Vector2i &v2) { return Vector2i(v1.x - v2.x, v1.y - v2.y); }
    friend Vector2i operator*(const int &a, const Vector2i &v) { return Vector2i(a * v.x, a * v.y); }
    friend Vector2i operator*(const Vector2i &v, const int &a) { return Vector2i(a * v.x, a * v.y); }

    // operateurs d'assignement

    friend void operator+=(Vector2i &v1, const Vector2i &v2)
    {
        v1.x += v2.x;
        v1.y += v2.y;
    }
    friend void operator-=(Vector2i &v1, const Vector2i &v2)
    {
        v1.x -= v2.x;
        v1.y -= v2.y;
    }
    friend void operator*=(Vector2i &v, const int &a)
    {
        v.x *= a;
        v.y *= a;
    }

    friend int dot(const Vector2i &v1, const Vector2i &v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    friend int dist(const Vector2i &v)
    {
        return sqrt(v.x * v.x + v.y * v.y);
    }
};