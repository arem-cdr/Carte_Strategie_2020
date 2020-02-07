#pragma once

#include <mbed.h>
#include <vector>
#include <iostream>

#include "BasicFunctions.h"
#include "Constantes.h"
#include "Point.h"
#include "Obstacle.h"

const float rotationCost = 20.f;

bool IntersectObs(const Vector2f &p1, const Vector2f &p2, const std::vector<Obstacle> &obstacles, const float &radiusObject); // const

class Node
{
private:
    Vector2f pos;

    std::vector<Node *> childrenNodes;
    Node *parentNode;

    float distToOrigin;

public:
    Node(const Vector2f pos = Vector2f(0.f, 0.f), Node *parentNode = NULL, const float distToOrigin = 0.f);
    ~Node();

    Vector2f getPos() const { return pos; }
    std::vector<Node *> getChildren() const { return childrenNodes; }
    int getNbChildren() const;
    Node *getParent() const { return parentNode; }
    float getDistToOrigin() const { return distToOrigin; }

    void setDistToOrigin(const float &d) { distToOrigin = d; }
    void setPos(const Vector2f &pos) { this->pos = pos; }
    void setParent(Node *parent) { this->parentNode = parent; }

    void addChild(const Vector2f posChild);
    void linkToTarget(Node &target);
    void detachTarget(Node &target);

    Node *detachChild(int indChild);

    void findClosestParcoursRec(Vector2f p, float &d, Node *&closest, const std::vector<Obstacle> &obstacles, float radiusObject);

    void displayNode() const;
    void QtdisplayNode(Serial *pc) const;
};

class RRT
{
private:
    Node *root;
    Node target;
    //std::vector<Node*> orphanTrees;

    std::vector<Vector2f> path;
    float pathLength;

    Node *findClosestNode(Vector2f p, const std::vector<Obstacle> &obstacles, float radiusObject) const;

    float evaluatePath(std::vector<Vector2f> path) const { return dist(path) + path.size() * 20.f; }

    void addNode(Vector2f pos, const std::vector<Obstacle> &obstacles, float radiusObject);

    void generateTree(int nbIterations, Point target, const std::vector<Obstacle> &obstacles, float radiusObject);
    void deleteRandomBranch();

    void updatePath(const Vector2f &targetPos, const std::vector<Obstacle> &obstacles, const float &radiusObject);
    bool isPathLegit(const Vector2f &objectPos, const Vector2f &targetPos, const std::vector<Obstacle> &obstacles, const float &radiusObject);

public:
    RRT(const Vector2f &rootPos = Vector2f(0.f, 0.f));
    ~RRT();

    Node *getRoot() const { return root; }

    void resetTree(Point rootPos);
    void resetPath();

    int countNodes() const;

    void maintainTree(Point objectPos, Point target, const std::vector<Obstacle> &obstacles, float radiusObject);

    void update(Point objectPos, Point target, const std::vector<Obstacle> &obstacles, float radiusObject); // fonction principale

    void display() const;
    void QtdisplayRRT(Serial *pc) const;
    void QtdisplayPath(Serial *pc) const;
};
