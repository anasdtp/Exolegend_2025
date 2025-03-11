#include "gladiator.h"
#include "Mathematiques/Mathematiques.h"
#include "Asservissement/Asservissement.h"
#include "GameData/GameData.h"
#include "Strategy/Strategy.h"

Gladiator *gladiator;
GameState *game;
StateMachine *statemachine;
Asservissement *motors;

bool match_started = false;

void reset();
void setup()
{
    // instanciation de l'objet gladiator

    gladiator = new Gladiator();

    motors = new Asservissement(gladiator);
    game = new GameState(gladiator, motors);
    statemachine = new StateMachine(game);
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset()
{
    // // fonction de reset:
    game->reset();
    match_started = false;

    // initialisation de toutes vos variables avant le début d'un match
    // gladiator->log("Call of reset function"); // GFA 4.5.1
    game->goal = gladiator->robot->getData().position;
    motors->setTargetPos(game->goal);
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        if (match_started == false)
        {
            game->start_time_match = millis();
            match_started = true;
        }
        game->Update();

        game->current_time = (millis() - game->start_time_match);

        if (TempsEchantionnage(TE_MS))
        {
            motors->positionControl(motors->getTargetPos());
        }

        statemachine->strategy();
        // robot_state_machine->machine();
    }
}
