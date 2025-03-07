#include "gladiator.h"
#include "Mathematiques/Mathematiques.h"
#include "Asservissement/Asservissement.h"
#include "GameData/GameData.h"

Gladiator *gladiator;
GameState *game;

Asservissement *motors;

int testPath[5][2] = {{1, 6}, {2,6}, {3, 6}, {4, 6}, {4, 6}};

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
    motors->setTargetPos(game->goal);

    for(int k = 0; k < 5; k++){
        int i = testPath[k][0];
        int j = testPath[k][1];
        game->gladiator->log("case à visitée :%d,%d", i, j);
        game->simplified_coord_list.path_coord[game->count].i = i;
        game->simplified_coord_list.path_coord[game->count].j = j;
        game->count = (game->count + 1)%max_parth_finder_size;
    }
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

