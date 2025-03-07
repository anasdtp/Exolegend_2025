#include "Mathematiques.h"

Position getSquareCoor(const MazeSquare *square, float squareSize)
{
    return getSquareCoor(square->i, square->j, squareSize);
}

Position getSquareCoor(uint8_t i, uint8_t j, float squareSize)
{
    Position coor;
    coor.x = (i + 0.5f) * squareSize;
    coor.y = (j + 0.5f) * squareSize;

    return coor;
}

MazeSquare *getMazeSquareCoor(const Position &pos, Gladiator *gladiator)
{
    uint8_t i = (uint8_t)(pos.x / gladiator->maze->getSquareSize());
    uint8_t j = (uint8_t)(pos.y / gladiator->maze->getSquareSize());

    return gladiator->maze->getSquare(i, j);;
}

float getDistance(const Position &p1, const Position &p2)
{
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

float getDistance(MazeSquare *s1, MazeSquare *s2)
{
    return sqrt(pow((s1->i - s2->i), 2) + pow((s1->j - s2->j), 2));
}

double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

// Heuristique : distance de Manhattan
uint8_t heuristic(MazeSquare *a, MazeSquare *b) {
    return abs(a->i - b->i) + abs(a->j - b->j);
}

