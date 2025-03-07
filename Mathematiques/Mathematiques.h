#ifndef MATHEMATIQUES_H
#define MATHEMATIQUES_H
#include "gladiator.h"

Position getSquareCoor(const MazeSquare *square, float squareSize);
Position getSquareCoor(uint8_t i, uint8_t j, float squareSize);
MazeSquare *getMazeSquareCoor(const Position &pos, Gladiator *gladiator);
float getDistance(const Position &p1, const Position &p2);
float getDistance(MazeSquare *s1, MazeSquare *s2);
double reductionAngle(double x);

// Heuristique : distance de Manhattan
uint8_t heuristic(MazeSquare *a, MazeSquare *b);


#endif // MATHEMATIQUES_H