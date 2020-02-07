#include "BasicFunctions.h"

bool segmentIntersectCircle(const Vector2f &p0, const Vector2f &p1, const Vector2f &c, const float &r)
{
    /** cette fonction est d'une importance capitale. Elle vérifie si il y a collision entre le segment [p0, p1] et
     * le cercle ce centre c et de rayon r. Si oui, elle renvoie true, sinon oui. Elle est la fonction principale à 
     * optimiser car appelée d'innombrables fois. Dans la plupart des cas, la fonction renvoie false, il faudrait donc
     * optimiser au maximum les performances lorsqu'il n'y a pas collision
     * */

    // légère optimisation permettant de gagner en performance : on verifie si le segment a des chances d'entrer en collision avec le point
    if (min(p0.getX(), p1.getX()) > c.getX() + r ||
        max(p0.getX(), p1.getX()) < c.getX() - r ||
        min(p0.getY(), p1.getY()) > c.getY() + r ||
        max(p0.getY(), p1.getY()) < c.getY() - r)
        return false;

    float dx = p1.getX() - p0.getX();
    float dy = p1.getY() - p0.getY();

    float Ox = p0.getX() - c.getX();
    float Oy = p0.getY() - c.getY();

    float A = dx * dx + dy * dy;
    float B = 2 * (dx * Ox + dy * Oy);
    float C = Ox * Ox + Oy * Oy - r * r;

    float delta = B * B - 4 * A * C;

    if (delta < 0)
        return false;

    float x1 = (-B + sqrt(delta)) / 2.f / A;
    float x2 = (-B - sqrt(delta)) / 2.f / A;

    if ((x1 >= 0.f && x1 <= 1.f) || (x2 >= 0.f && x2 <= 1.f))
    {
        return true;
    }

    return false;
}

float dist(std::vector<Vector2f> vector)
{
    float d = 0;
    for (size_t i = 0; i < vector.size(); i++)
    {
        d += dist(vector[i]);
    }

    return d;
}