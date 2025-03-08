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
        EXPLORE_BOMB,
        SURVIVAL,
        EXPLORE,
        ATTACK,
        EVADE
    };

    GameState *game;
    State currentState;

    int etat_exploration;

    StateMachine(GameState *game);

    void Update();

    void reset();

    void strategy();

    bool CloseEnemy(float dist_thresh);

    bool CloseMaxWall();

    bool TimeToExplode();

    MazeSquare *getBestBomb();

    MazeSquare *getSafeSquare();
};

#endif // STRATEGY_H