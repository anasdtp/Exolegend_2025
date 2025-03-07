#include "Strategy.h"
#include "Asservissement/Asservissement.h"
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
    if (game->current_time % 20000 < 5000) // On se dirige vers le centre du labyrinthe si il reste 5 secondes avant le prochain retrécissement
    {
        Position current_pos = game->gladiator->robot->getData().position;
        float next_wall_size = game->gladiator->maze->getCurrentMazeSize() - 0.2f;
        float min_x = (3 - next_wall_size) / 2, max_x = 3 - min_x, min_y = (3 - next_wall_size) / 2, max_y = 3 - min_y;
        if (current_pos.x < min_x || current_pos.x > max_x || current_pos.y < min_y || current_pos.y > max_y)
        {
            near = true;
        }
    }

    return near;
}

bool StateMachine::TimeToExplode()
{
    return (0);
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
        {
            MazeSquare *current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator);
            uint8_t sg_x = 1;
            uint8_t sg_y = 1;
            if (current_square->i > 6)
            {
                sg_x = -1;
            }
            if (current_square->j > 6)
            {
                sg_y = -1;
            }
            MazeSquare target_square = {current_square->i + sg_x, current_square->j + sg_y};
            game->gotoSquare(&target_square);
        }
        break;

        case State::EXPLORE:
        {
        }
        break;
        }
    }
}
