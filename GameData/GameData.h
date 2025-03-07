#pragma once
#include "gladiator.h"
#include "Asservissement/Asservissement.h"
#ifndef max_parth_finder_size
#define max_parth_finder_size 32
#endif


struct coordonnees
{
    int i;
    int j;
};

struct pathFinder
{
    int size;
    coordonnees path_coord[max_parth_finder_size];
};

class GameState
{
public:
    Gladiator *gladiator;
    Asservissement *motors;

    RobotData myData;
    RobotData allyData;
    RobotData er1Data;
    RobotData er2Data;

    Position goal;

    float squareSize;

    int count;
    pathFinder coord_list;
    pathFinder simplified_coord_list;

    GameState(Gladiator *gladiator, Asservissement *motors);

    void Update();
    void reset();

    void gotoSquare(MazeSquare *square);
};