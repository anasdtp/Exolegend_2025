#include "Mathematiques.h"

Position getSquareCoor(MazeSquare *square, float squareSize)
{
    // if(square == nullptr){
    //     return {0, 0, 0};
    // }
    Position coor;
    coor.x = (float(square->i) + 0.5f) * squareSize;
    coor.y = (float(square->j) + 0.5f) * squareSize;

    return coor;
}


// Thread 1 "program" received signal SIGSEGV, Segmentation fault.
// 0x000055555556b09e in getSquareCoor (square=0x0, squareSize=0.252299994) at src/Mathematiques/Mathematiques.cpp:5
// 5	    return getSquareCoor(square->i, square->j, squareSize);
// (gdb) bt
// #0  0x000055555556b09e in getSquareCoor (square=0x0, squareSize=0.252299994) at src/Mathematiques/Mathematiques.cpp:5
// #1  0x000055555556781e in simpleAStar (gladiator=0x55555565b3f0, current_square=0x55555565e6f8, target_square=0x0) at src/AStar/AStar.cpp:94
// #2  0x000055555556c35a in StateMachine::strategy (this=0x555555660790) at src/Strategy/Strategy.cpp:224
// #3  0x000055555556c831 in loop () at src/main.cpp:54
// #4  0x000055555558f746 in main ()


Position getSquareCoor(uint8_t i, uint8_t j, float squareSize)
{
    Position coor;
    coor.x = (float(i) + 0.5f) * squareSize;
    coor.y = (float(j) + 0.5f) * squareSize;

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
    x = fmod(x + PI_F, 2 * PI_F);
    if (x < 0)
        x += 2 * PI_F;
    return x - PI_F;
}

// Heuristique : distance de Manhattan
uint8_t heuristic(MazeSquare *a, MazeSquare *b) {
    return abs(a->i - b->i) + abs(a->j - b->j);
}

