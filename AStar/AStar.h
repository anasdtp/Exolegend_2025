#ifndef ASTAR_H
#define ASTAR_H
#include "gladiator.h"
#include "Mathematiques/Mathematiques.h"
#include <cmath>
#include <cstring> // For memset

struct SimplePath
{
    Position steps[144]; // Max path length for 12x12 maze
    int length = 0;
};

// Open list (stores nodes to explore)
struct Node
{
    byte i, j;
    float total_cost;
};
float complete_heurisic(Position start, Position target);

SimplePath simpleAStar(Gladiator *gladiator, MazeSquare *current_square, MazeSquare *targer_square);




#endif // ASTAR_H