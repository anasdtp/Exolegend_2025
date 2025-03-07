#include "Strategy.h"
#include "Asservissement/Asservissement.h"

StateMachine::StateMachine(GameState *game) {
    this->game = game;

    currentState = State::ATTENTE;
    etat_exploration = 0;
}

void StateMachine::reset()
{
    currentState = State::EXPLORATION;
    etat_exploration = 0;
}

bool StateMachine::closeEnnemi(float dist_thresh)
{
    bool near = false;
    RobotData my_data = game->gladiator->robot->getData();

    RobotList ids_list = game->gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; i++)
    {

        if (ids_list.ids[i] != 121 && ids_list.ids[i] != 120)
        {
            RobotData others_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
            if (getDistance(my_data.position,others_data.position) < dist_thresh)
            {
                near = true;
            }
        }
    }
    return near;
}

void StateMachine::strategy()
{
    bool t_recherche_fusee;// = !game->gladiator->weapon->canLaunchRocket();
    bool t_ennemi_proche;// = ennemi_proche(game->gladiator);
    bool t_recherche_cible = true;
    bool t_tirer = true;
    // game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);

    switch (currentState)
    {
    case State::ATTENTE:
        if (t_ennemi_proche)
        {
            currentState = State::PVP;
        }
        else if (t_recherche_fusee)
        {
            currentState = State::RECHERCHE_FUSEE;
        }
        else
        {
            currentState = State::EXPLORATION;
        }
        break;

    case State::RECHERCHE_FUSEE:
        // new_missile(game);
        currentState = State::ATTENTE;

        break;

    case State::EXPLORATION:
    {
        switch (etat_exploration)
        {
        case 0:
        {
            int testPath[5][2] = {{1, 7}, {3, 5}, {2, 3}, {3, 4}, {4, 5}};
            for(int k = 0; k < 1; k++){
                int i = testPath[k][0];
                int j = testPath[k][1];
                game->gladiator->log("case à visitée :%d,%d", i, j);
                game->simplified_coord_list.path_coord[game->count].i = i;
                game->simplified_coord_list.path_coord[game->count].j = j;
                game->count = (game->count + 1)%max_parth_finder_size;
            }
            etat_exploration = 1;
        }
        break;
        case 1 :
        {
            // if(!followPath(game)){
            //     etat_exploration = 2;
            // }
        }
        break;
        case 2:
        {
            etat_exploration = 2;
        }
        break;
        default:
            etat_exploration = 0;
            break;
        }
        
        // if (t_recherche_cible)
        // {
        //     currentState = State::RECHERCHE_CIBLE;
        // }
        // if (t_ennemi_proche)
        // {
        //     currentState = State::PVP;
        // }
        // else
        // {
        //     currentState = State::ATTENTE;
        // }
    }
    break;

    case State::PVP:
        // executePVP();
        currentState = State::ATTENTE;
        break;

    case State::RECHERCHE_CIBLE:
        t_recherche_cible = false;
        if (t_tirer)
        {
            currentState = State::TIRER;
        }
        else
        {
            currentState = State::EXPLORATION;
        }
        break;

    case State::TIRER:
        // game->gladiator->weapon->launchRocket();
        currentState = State::ATTENTE;
        break;
    }
}
// void StateMachine::executePVP()
// {
//     RobotData my_data = game->gladiator->robot->getData();
//     RobotData ally_data;
//     RobotList ids_list = game->gladiator->game->getPlayingRobotsId();
//     for (int i = 0; i < 4; i++)
//     {
//         if ((ids_list.ids[i] == 121 || ids_list.ids[i] != 120) && ids_list.ids[i] != my_data.id)
//         {
//             ally_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
//         }
//     }
//     SpartanMode(game);
// }

