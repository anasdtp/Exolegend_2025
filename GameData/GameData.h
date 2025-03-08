#pragma once
#include "gladiator.h"
#include "Asservissement/Asservissement.h"
#ifndef max_parth_finder_size
#define max_parth_finder_size 32
#endif

#define SIZE 12

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

    unsigned long start_time_match; // En millisecondes
    unsigned long current_time;     // En millisecondes

    float squareSize;

    MazeSquare *maze[SIZE][SIZE];

    GameState(Gladiator *gladiator, Asservissement *motors);

    void Update();
    void reset();

    void gotoSquare(MazeSquare *square);
};