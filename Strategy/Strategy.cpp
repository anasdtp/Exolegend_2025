#include "Strategy.h"
#include "Asservissement/Asservissement.h"
#include "Mathematiques/Mathematiques.h"
#include "AStar/AStar.h"

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
        // game->gladiator->log("void StateMachine::CloseMaxWall() : Prochain retrécissement dans 3 secondes");
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

MazeSquare *StateMachine::getNearestBomb()
{
    float min_dist = 1000;
    MazeSquare *current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator);
    MazeSquare *nearest_bomb = nullptr;
    const int searchRadius = 5; // On ne regarde que dans un carré de 5 cases autour de la position actuelle

    // Définir les bornes de la recherche en fonction des indices du carré courant
    uint8_t i_min = (current_square->i >= searchRadius ? current_square->i - searchRadius : 0);
    uint8_t i_max = (current_square->i + searchRadius < SIZE ? current_square->i + searchRadius : SIZE - 1);
    uint8_t j_min = (current_square->j >= searchRadius ? current_square->j - searchRadius : 0);
    uint8_t j_max = (current_square->j + searchRadius < SIZE ? current_square->j + searchRadius : SIZE - 1);

    for (uint8_t i = i_min; i <= i_max; i++)
    {
        for (uint8_t j = j_min; j <= j_max; j++)
        {
            MazeSquare *square = game->gladiator->maze->getSquare(i, j);
            if (square->coin.value) // La case contient une bombe (ou coin, selon ta logique)
            {
                float dist = getDistance(current_square, square);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest_bomb = square;
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
    int number_of_bombs = game->gladiator->weapon->getBombCount();
    // bool f_time_to_explode = TimeToExplode();

    // game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);

    switch (currentState)
    {
    case State::WAIT:
    {
        if (number_of_bombs)
        {
            game->gladiator->weapon->dropBombs(number_of_bombs);
        }

        if (f_close_max_wall)
        {
            currentState = State::SURVIVAL;
        }
        else if (game->motors->available())
        {
            // if (f_close_enemy)
            // {
            //     currentState = State::CLOSE_ENEMY;
            // }

            // else if (f_time_to_explode)
            // {
            //     currentState = State::EXPLOSION;
            // }
            // else
            if (bomb)
            {
                currentState = State::EXPLORE_BOMB;
            }
            else
            {
                currentState = State::EXPLORE;
            }
        }
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
        MazeSquare *nearest_bomb = getNearestBomb();
        game->gotoSquare(nearest_bomb);
        currentState = State::WAIT;
    }
    break;

    case State::EXPLORE:
    {
        // On cherche où sont les bombes les plus proches et on s'y dirige et on les ramasse puis explose
        MazeSquare *current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator); // me donne les distances en mètres
        MazeSquare *nearest_bomb = getNearestBomb();

        SimplePath path = simpleAStar(game->gladiator, current_square, nearest_bomb);
        if (path.length > 0)
        {
            if (path.length > 1)
            {
                // Move robot through path.steps[0] to path.steps[path.length-1] // Remlacer par path.length-1 pour aller direct sur les bombes (pas besoin du else dans ce cas)
                MazeSquare *nextPos = getMazeSquareCoor(path.steps[1], game->gladiator);
                game->gotoSquare(nextPos);
            }
            else
            {
                MazeSquare *nextPos = getMazeSquareCoor(path.steps[0], game->gladiator);
                game->gotoSquare(nextPos);
            }
        }
        currentState = State::WAIT;
    }
    break;
    }
}
