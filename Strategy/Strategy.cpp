#include "Strategy.h"
#include "Asservissement/Asservissement.h"
#include "Asservissement/goto.h"
#include "Mathematiques/Mathematiques.h"

StateMachine::StateMachine(GameState *game)
{
    this->game = game;

    currentState = State::WAIT;
    etat_exploration = 0;
}

void StateMachine::reset()
{
    currentState = State::EXPLORE;
    etat_exploration = 0;
}

bool StateMachine::CloseEnemy(float dist_thresh)
{
    bool near = false;
    RobotData my_data = game->gladiator->robot->getData();

    RobotList ids_list = game->gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; i++)
    {

        if (ids_list.ids[i] != 121 && ids_list.ids[i] != 120)
        {
            RobotData others_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
            if (getDistance(my_data.position, others_data.position) < dist_thresh)
            {
                near = true;
            }
        }
    }
    return near;
}

bool StateMachine::CloseMaxWall()
{
    bool near = false;
    // On récupère les coordonnées max du labyrinthe, si il reste 5 secondes avant le prochain retrécissement
    // On se dirige vers le centre du labyrinthe
    // Le terrain rétrécit de 1 cellule à droite, à gauche, en haut et en bas toutes les 20 secondes.
    if time_left
        < 5
        {
            near = true;

            float next_wall_size = game->gladiator->maze->getCurrentMazeSize() - 0.2f;
            float min_x = (3 - next_wall_size) / 2, max_x = 3 - min_x, min_y = (3 - next_wall_size) / 2, max_y = 3 - min_y;
            if (game->gladiator->robot->getData().position.x < min_x || game->gladiator->robot->getData().position.x > max_x || game->gladiator->robot->getData().position.y < min_y || game->gladiator->robot->getData().position.y > max_y)
            {
                near = true;
            }
        }
    return near;
}

bool StateMachine::TimeToExplode()
{
}

void StateMachine::strategy()
{
    bool f_close_enemy = CloseEnemy(0.5);
    bool f_close_max_wall = CloseMaxWall();
    bool f_time_to_explode = TimeToExplode();
    // game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);

    switch (currentState)
    {
    case State::WAIT:
        if (f_close_enemy)
        {
            currentState = State::CLOSE_ENEMY;
        }
        else if (f_close_max_wall)
        {
            currentState = State::SURVIVAL;
        }
        else if (f_time_to_explode)
        {
            currentState = State::EXPLOSION;
        }
        else
        {
            currentState = State::EXPLORE;

            break;

        case State::SURVIVAL:
            Position current_pos = game->gladiator->robot->getData().position;
            float sg_x = 1;
            float sg_y = 1;
            if (current_pos.x > 1.5)
            {
                sg_x = -1;
            }
            if (current_pos.y > 1.5)
            {
                sg_y = -1;
            }
            Position target_pos = {current_pos.x + sg_x * 0.2f, current_pos.y + sg_y * 0.2f, current_pos.a};
            game->motors->setTargetPos(target_pos);
            break;

        case State::EXPLORE:
        {
            switch (etat_exploration)
            {
            case 0:
            {
                int testPath[5][2] = {{1, 7}, {3, 5}, {2, 3}, {3, 4}, {4, 5}};
                for (int k = 0; k < 1; k++)
                {
                    int i = testPath[k][0];
                    int j = testPath[k][1];
                    game->gladiator->log("case à visitée :%d,%d", i, j);
                    game->simplified_coord_list.path_coord[game->count].i = i;
                    game->simplified_coord_list.path_coord[game->count].j = j;
                    game->count = (game->count + 1) % max_parth_finder_size;
                }
                etat_exploration = 1;
            }
            break;
            case 1:
            {
                if (!followPath(game))
                {
                    etat_exploration = 2;
                }
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
        }
    }
