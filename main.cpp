#include "gladiator.h"
#include "Asservissement/Asservissement.h"
#include "GameData/GameData.h"

Gladiator *gladiator;
GameState *game;

Asservissement *motors;

void reset();
void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    motors = new Asservissement(gladiator);
    game = new GameState(gladiator, motors);
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void reset()
{
    // fonction de reset:
    game->reset();
    
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
    game->goal = gladiator->robot->getData().position;
    Position target = {game->goal.x, game->goal.y +1, 0};
    motors->setTargetPos(target);
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        game->Update();

        if (TempsEchantionnage(TE_MS))
        {
            motors->positionControl(motors->getTargetPos());
        }
        // robot_state_machine->machine();

    }
}
