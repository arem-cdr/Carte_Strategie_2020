#pragma once

#include "Vector2i.h"

const float PI = 3.141593f;

enum team
{
    NONE = 0,
    BLUE,
    YELLOW
};

const int tableLength = 3000; //en cm
const int tableWidth = 2000;

const int gobeletRadius = 37;
const int robotRadius = 160;
const int robotHalfWidth = 135;

// Cout d'une rotation pour l'évaluation d'un chemin
const float rotationCost = 200.f;
const float basePathLength = 50000.f;

// position des cercles repésentant les tasseaux du terrain
const Vector2i tasseauCentral1(1500, 1700);
const Vector2i tasseauCentral2(1500, 1850);
const Vector2i tasseauBleu(900, 1850);
const Vector2i tasseauJaune(2100, 1850);