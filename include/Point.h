#pragma once
#include "constantes.h"
#include "Vector2f.h"

class Point
{
private:
    Vector2f pos;
    float angle;

public:
    //Constructor/destructor
    Point(float x = 0.f, float y = 0.f, float angle = 0.f)
    {
        this->pos = Vector2f(x, y);
        this->angle = angle;
    }
    Point(Vector2f pos, float angle = 0.f) : pos(pos), angle(angle) {}
    ~Point() {}

    //Accessors
    Vector2f getPos() const { return pos; }
    float getangle() const { return angle; }

    //Set
    //void setX(const float x) { this->x = x; }
    //void setY(const float y) { this->y = y; }
    void Setangle(const float angle) { this->angle = angle; }
    void Setall(const float x, const float y, const float angle)
    {
        this->pos = Vector2f(x, y);
        this->angle = angle;
    }

    // operateur d'affichage
    friend std::ostream &operator<<(std::ostream &flux, const Point &p)
    {
        std::cout << p.pos << " " << p.angle << std::endl;
        return flux;
    }

    // operateurs de comparaison
    friend bool operator==(const Point &p1, const Point &p2) { return p1.pos == p2.pos && p1.angle == p2.angle; }
    friend bool operator!=(const Point &p1, const Point &p2) { return !(p1 == p2); }

    // operateurs arithmetiques
    friend Point operator+(const Point &p1, const Point &p2) { return Point(p1.pos + p2.pos, p1.angle + p2.angle); }
    friend Point operator-(const Point &p1, const Point &p2) { return Point(p1.pos - p2.pos, p1.angle - p2.angle); }

    friend void operator+=(Point &p1, const Point &p2)
    {
        p1.pos += p2.pos;
        p1.angle += p2.angle;
    }
    friend void operator+=(Point &p1, const Vector2f &v2)
    {
        p1.pos += v2;
    }
    friend void operator-=(Point &p1, const Point &p2)
    {
        p1.pos -= p2.pos;
        p1.angle -= p2.angle;
    }
    friend void operator-=(Point &p1, const Vector2f &v2)
    {
        p1.pos -= v2;
    }
};
