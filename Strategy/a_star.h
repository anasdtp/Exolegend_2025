#include "gladiator.h"
#include "GameData/GameData.h"
#include "Mathematiques/Mathematiques.h"
// AStar.h
#pragma once
#include <cmath>
#include <cstring> // For memset

struct SimplePath
{
    Position steps[144]; // Max path length for 12x12 maze
    int length;
};

float complete_heurisic(Position start, Position target);

SimplePath simpleAStar(Gladiator *gladiator, Position start, Position target);