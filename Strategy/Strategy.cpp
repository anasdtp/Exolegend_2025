#include "Strategy.h"
#include "Asservissement/Asservissement.h"
#include "Mathematiques/Mathematiques.h"
#include "a_star.h"

StateMachine::StateMachine(GameState *game)
{
    this->game = game;

    currentState = State::WAIT;
    etat_exploration = 0;
}

void StateMachine::reset()
{
    currentState = State::WAIT;
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
    if (game->current_time % 20000 > 17000) // On se dirige vers le centre du labyrinthe si il reste 5 secondes avant le prochain retrécissement
    {
        Position current_pos = game->gladiator->robot->getData().position;
        float next_wall_size = game->gladiator->maze->getCurrentMazeSize() - 0.5f;
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

Position StateMachine::getNearestBomb()
{
    Position current_pos = game->gladiator->robot->getData().position;
    Position nearest_bomb = current_pos;
    float min_dist = 1000;
    MazeSquare *current_square = getMazeSquareCoor(current_pos, game->gladiator);
    for (uint8_t i; i < 12; i++)
    {
        for (uint8_t j; j < 12; j++)
        {
            MazeSquare *square = game->gladiator->maze->getSquare(i, j);
            if (square->coin.value)
            {
                Position bomb_pos = getSquareCoor(square, game->gladiator->maze->getSquareSize());
                float dist = getDistance(current_pos, bomb_pos);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_bomb = bomb_pos;
                }
            }
        }
    }

    return nearest_bomb;
}

void StateMachine::strategy()
{

    // bool f_close_enemy = CloseEnemy(0.5);
    bool f_close_max_wall = CloseMaxWall();
    bool bomb = false;
    // bool f_time_to_explode = TimeToExplode();

    // game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);

    switch (currentState)
    {
    case State::WAIT:
        // if (f_close_enemy)
        // {
        //     currentState = State::CLOSE_ENEMY;
        // }
        if (f_close_max_wall)
        {
            currentState = State::SURVIVAL;
        }
        else if (bomb)
        {
            currentState = State::EXPLORE_BOMB;
        }
        else
        {
            currentState = State::EXPLORE;
        }
        break;

    case State::SURVIVAL:
    {
        MazeSquare *current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator);
        int sg_x = 1;
        int sg_y = 1;
        if (current_square->i > 6)
        {
            sg_x = -1;
        }
        if (current_square->j > 6)
        {
            sg_y = -1;
        }
        MazeSquare target_square = {uint8_t(int(current_square->i) + sg_x), uint8_t(int(current_square->j) + sg_y)};
        game->gotoSquare(&target_square);
        currentState = State::WAIT;
    }
    break;

    case State::EXPLORE_BOMB:
    {
        // On cherche où sont les bombes les plus proches et on s'y dirige et on les ramasse puis explose
        Position nearest_bomb = getNearestBomb();
        game->gotoSquare(getMazeSquareCoor(nearest_bomb, game->gladiator));
        currentState = State::WAIT;
    }
    break;

    case State::EXPLORE:
    {
        // On cherche où sont les bombes les plus proches et on s'y dirige et on les ramasse puis explose
        Position currentPos = game->gladiator->robot->getData().position;
        Position target = getNearestBomb();

        Path path = aStarPathfinding(game->gladiator, currentPos, target);
        if (path.size > 0)
        {
            game->gladiator->log("Path found with %s", String(path.size), " steps");
            currentState = State::WAIT;
        }
        break;
    }
    }
}
