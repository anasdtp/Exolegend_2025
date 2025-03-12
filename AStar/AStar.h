#ifndef ASTAR_H
#define ASTAR_H
#include "gladiator.h"
#include "GameData/GameData.h"
#include "Mathematiques/Mathematiques.h"
#include <cmath>
#include <cstring> // For memset

typedef struct
{
    Position steps[144]; // Max path length for 12x12 maze
    int length = 0;
}SimplePath;

// Open list (stores nodes to explore)
typedef struct 
{
    uint8_t i, j;
    float total_cost;
}Node;

float complete_heurisic(Position start, Position target);

SimplePath simpleAStar(GameState *game, MazeSquare *current_square, MazeSquare *targer_square);
SimplePath simplifyPath(const SimplePath &originalPath);



#endif // ASTAR_H