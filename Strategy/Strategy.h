#ifndef STRATEGY_H
#define STRATEGY_H

#include "gladiator.h"
#include "GameData/GameData.h"
#include "Mathematiques/Mathematiques.h"

class StateMachine
{
public:
    enum class State
    {
        ATTENTE,
        RECHERCHE_FUSEE,
        EXPLORATION,
        PVP,
        RECHERCHE_CIBLE,
        TIRER
    };

    GameState *game;
    State currentState;

    int etat_exploration;

    StateMachine(GameState *game);

    void reset();

    void strategy();
    
    bool closeEnnemi(float dist_thresh);


};


#endif // STRATEGY_H