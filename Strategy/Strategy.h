#ifndef STRATEGY_H
#define STRATEGY_H

#include "gladiator.h"
#include "GameData/GameData.h"
#include "Mathematiques/Mathematiques.h"

class StateMachine
{
public:
    enum State
    {
        WAIT,
        SURVIVAL,
        EXPLORE,
        ATTACK,
        EVADE
    };

    GameState *game;
    State currentState;
    MazeSquare *square;

    MazeSquare *target_square;
    MazeSquare *current_square;
    MazeSquare *nearest_bomb;
    MazeSquare *nextPos;
    MazeSquare *neighbors_strat[5];
    MazeSquare *bomb_square;
    int sum_threshold;

    int etat_exploration;

    StateMachine(GameState *game);

    void Update();

    void reset();

    void strategy();

    bool CloseEnemy(uint8_t dist_thresh);

    bool CloseDeadEnemy(uint8_t dist_thresh);

    bool CloseMaxWall();

    bool TimeToExplode();

    MazeSquare *getBestBomb();

    MazeSquare *getSafeSquare();

    MazeSquare *nearestOpponent();
    
    void nearestOpponent(Position &pos);

};

#endif // STRATEGY_H