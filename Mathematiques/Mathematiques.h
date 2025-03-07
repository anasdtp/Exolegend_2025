#ifndef MATHEMATIQUES_H
#define MATHEMATIQUES_H
#include "gladiator.h"

Position getSquareCoor(const MazeSquare *square, float squareSize);
Position getSquareCoor(uint8_t i, uint8_t j, float squareSize);
float getDistance(const Position &p1, const Position &p2);
double reductionAngle(double x);


#endif // MATHEMATIQUES_H