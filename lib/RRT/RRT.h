#pragma once

#include <mbed.h>
#include <vector>
#include <iostream>

#include "RRTConstantes.h"
#include "Point.h"
#include "Obstacle.h"

/** 
 * Renvoie true si le segment [p1, p2] intersecte un des obstacle, 
 * représenté par un cercle de rayon rayon de l'obstacle + rayon de l'objet
 */
bool IntersectObs(const Vector2i &p1,
                  const Vector2i &p2,
                  const std::vector<Obstacle *> &obstacles,
                  const int &radiusObject,
                  const int &halfWidthObject);

/**
 * Renvoie true le segment [p1, p2] intersecte le cercle de centre c et de rayon r 
 */
bool segmentIntersectCircle(const Vector2i &p1, const Vector2i &p2, const Vector2i &c, const int &r);

/**
 * Renvoie true le segment [p1, p2] intersecte le cercle de centre c et de rayon r
 * On assimile ici le cercle à une croix, la fonction va donc seulement évaluer s'il
 * y a collision entre les deux segments de la croix et [p1, p2] 
 */
bool segmentIntersectSimplifiedCircle(const Vector2i &p1, const Vector2i &p2, const Vector2i &c, const int &r);

/**
 * Renvoie la somme des normes de tous les vector2f de vector 
 */
float dist(const std::vector<Vector2i> &vector);

class Node
{
private:
    Vector2i pos;

    std::vector<Node *> childrenNodes;
    Node *parentNode;

    float distToOrigin;

public:
    Node(const Vector2i pos = Vector2i(0.f, 0.f), Node *parentNode = NULL, const float distToOrigin = 0.f);
    ~Node();

    // getters

    Vector2i getPos() const { return pos; }
    std::vector<Node *> getChildren() const { return childrenNodes; }
    int getNbChildren() const;
    Node *getParent() const { return parentNode; }
    float getDistToOrigin() const { return distToOrigin; }

    // setters

    void setDistToOrigin(const float &d) { distToOrigin = d; }
    void setPos(const Vector2i &pos) { this->pos = pos; }
    void setParent(Node *parent) { this->parentNode = parent; }

    // méthodes

    void addChild(const Vector2i &posChild);
    void linkToTarget(Node &target);
    void detachTarget(Node &target);
    Node *detachChild(int indChild);

    // fonction recursive pour trouver le node le plus proche du point p
    void findClosestParcoursRec(const Vector2i &p,
                                float &shortestDist,
                                Node *&closest,
                                const std::vector<Obstacle *> &obstacles,
                                const int &radiusObject,
                                const int &halfWidthObject);

    void displayNode() const;
    void QtdisplayNode(Serial *pc) const;
};

class RRT
{
private:
    Node *root;
    Node target;

    std::vector<Vector2i> path;
    float pathLength;

    Node *findClosestNode(const Vector2i &p, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject) const;

    float evaluatePath(const std::vector<Vector2i> &path) const { return dist(path) + path.size() * 200.f; }

    void addNode(const Vector2i &pos, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject);

    void generateTree(const int nbIterations,
                      const Point &target,
                      const std::vector<Obstacle *> &obstacles,
                      const int &radiusObject,
                      const int &halfWidthObject);
    void deleteRandomBranch();

    void updatePath(const Vector2i &targetPos, const std::vector<Obstacle *> &obstacles, const int &radiusObject, const int &halfWidthObject);
    bool isPathLegit(const Vector2i &objectPos,
                     const Vector2i &targetPos,
                     const std::vector<Obstacle *> &obstacles,
                     const int &radiusObject,
                     const int &halfWidthObject);

public:
    RRT(const Vector2i &rootPos = Vector2i(0.f, 0.f));
    ~RRT();

    Node *getRoot() const { return root; }

    void resetTree(const Point &rootPos);
    void resetPath();

    int countNodes() const;

    // supprime les branches qui ne sont plus valides avec la position du robot et des obstacles mis à jour
    void maintainTree(const Point &objectPos,
                      const Point &target,
                      const std::vector<Obstacle *> &obstacles,
                      const int &radiusObject,
                      const int &halfWidthObject);
    // developpe l'arbre en utilisant tout le temps imparti
    void update(const Point &objectPos,
                const Point &target,
                const std::vector<Obstacle *> &obstacles,
                const int &radiusObject,
                const int &halfWidthObject,
                const float &timeAvailable);

    void mainLoop(const Point &objectPos,
                  const Point &target,
                  const std::vector<Obstacle *> &obstacles,
                  const int &radiusObject,
                  const int &halfWidthObject,
                  const float &timeAvailable);

    // fonctions de debug/affichage
    void display() const;
    void QtdisplayRRT(Serial *pc) const;
    void QtdisplayPath(Serial *pc) const;
};
