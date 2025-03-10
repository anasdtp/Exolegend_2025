#include "Strategy.h"
#include "Asservissement/Asservissement.h"
#include "Mathematiques/Mathematiques.h"
#include "AStar/AStar.h"

StateMachine::StateMachine(GameState *game)
{
    this->game = game;

    currentState = State::EXPLORE;
    etat_exploration = 0;
    square = nullptr;
    data = game->gladiator->robot->getData();
    target_square = nullptr;
    current_square = nullptr;
    nearest_bomb = nullptr;
    nextPos = nullptr;
    bomb_square = nullptr;
    neighbors_strat[4] = square;
    neighbors_strat[0] = nullptr;
    neighbors_strat[1] = nullptr;
    neighbors_strat[2] = nullptr;
    neighbors_strat[3] = nullptr;
    sum_threshold = 3;
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
        if (ids_list.ids[i] != game->myData.id && ids_list.ids[i] != game->allyData.id)
        {
            RobotData others_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
            if (getDistance(my_data.position, others_data.position) < dist_thresh && others_data.lifes != 0)
            {
                near = true;
            }
        }
    }
    return near;
}

bool StateMachine::CloseDeadEnemy(float dist_thresh)
{
    bool near = false;
    RobotData my_data = game->gladiator->robot->getData();

    RobotList ids_list = game->gladiator->game->getPlayingRobotsId();
    for (int i = 0; i < 4; i++)
    {
        if (ids_list.ids[i] != game->myData.id && ids_list.ids[i] != game->allyData.id)
        {
            RobotData others_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
            if (getDistance(my_data.position, others_data.position) < dist_thresh && others_data.lifes == 0)
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

MazeSquare *StateMachine::getBestBomb()
{
    float max_score = -1000;
    this->current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator);
    this->nearest_bomb = getMazeSquareCoor(Position{6, 6, 0}, game->gladiator);

    int next_maze_size = int(game->gladiator->maze->getCurrentMazeSize() / game->gladiator->maze->getSquareSize());
    int min_index = 0, max_index = 11;
    min_index = (12 - next_maze_size) / 2, max_index = 11 - min_index;
    if (game->current_time % 20000 > 10000) // On se dirige vers le centre du labyrinthe si il reste 7 secondes avant le prochain retrécissement
    {
        min_index++;
        max_index--;
    }

    for (uint8_t i = min_index; i <= max_index; i++)
    {
        for (uint8_t j = min_index; j <= max_index; j++)
        // for (uint8_t i = min_index + 1; i < max_index; i++)
        // {
        //     for (uint8_t j = min_index + 1; j < max_index; j++)
        {
            bomb_square = game->gladiator->maze->getSquare(i, j);

            if (bomb_square->coin.value) // La case contient une bombe (ou coin, selon ta logique)
            {
                // Obtenir la case du centre du terrain
                // Calcul du score de la case en fonction de la distance, de la présence d'une bombe, et de la présence d'une obstacle
                // La présence d'une obstacle augmente le score, la présence d'une bombe diminue le score, et la distance diminue le score
                // La présence d'une bombe en cours diminue le score, et la présence d'une bombe non en cours augmente le score
                // La présence d'une bombe non en cours diminue le score, et la présence d'une bombe en cours augmente le score
                float score = -getDistance(current_square, bomb_square) + bomb_square->coin.value * 0.5f - bomb_square->danger * 1.0f - (bomb_square->possession == game->gladiator->robot->getData().teamId) * 1.0f + getDistance(bomb_square, game->gladiator->maze->getSquare(6, 6)) * 0.5f;
                if (score > max_score)
                {
                    max_score = score;

                    this->nearest_bomb = bomb_square;
                }
            }
        }
    }

    return this->nearest_bomb;
}

MazeSquare *StateMachine::getSafeSquare()
{
    // 1ère méthode : faire une recherche en cerclen autour du robot, en ne regardant que les cases valables
    current_square = game->getCurrentSquare();

    int next_maze_size = int(game->gladiator->maze->getCurrentMazeSize() / 0.25);
    int min_index = (12 - next_maze_size) / 2, max_index = 11 - min_index;

    for (int i = 1; i < 5; i++)
    {
        int steps = 16; // Adjust based on precision needed
        for (int k = 0; k < steps; k++)
        {
            float theta = k * (2 * PI / steps);
            int dx = int(cos(theta) * i);
            int dy = int(sin(theta) * i);
            MazeSquare *new_square = game->gladiator->maze->getSquare(current_square->i + dx, current_square->j + dy);
            if (new_square != nullptr && new_square->danger == 0 && new_square->i >= min_index && new_square->j >= min_index && new_square->i <= max_index && new_square->j <= max_index)
            {
                game->gladiator->log("found square with radius");
                return new_square;
            }
        }
    }

    // Backup quand même ; il s'agit du code utilisé précédemment pour rentrer dans les limites du terrain
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

    current_square->i = uint8_t(int(current_square->i) + sg_x);
    current_square->j = uint8_t(int(current_square->j) + sg_y);
    return current_square;
}

Position StateMachine::locateOpponents(Gladiator *gladiator)
{
    RobotData my_data = game->gladiator->robot->getData();
    Position enemy_pos;
    float min_dist = 100;
    // Récupérer la liste des IDs des robots en jeu
    RobotList ids_list = game->gladiator->game->getPlayingRobotsId();

    // Parcourir chaque robot et récupérer ses données
    for (int i = 0; i < 4; i++)
    {

        if (ids_list.ids[i] != game->myData.id && ids_list.ids[i] != game->allyData.id)
        {
            RobotData others_data = game->gladiator->game->getOtherRobotData(ids_list.ids[i]);
            if (getDistance(my_data.position, others_data.position) < min_dist)
            {
                enemy_pos = others_data.position;
            }
        }
    }

    return enemy_pos;
}

void StateMachine::strategy()
{
    // Variables de gestion des ennemis
    bool close_to_enemy = CloseEnemy(game->squareSize * 2.f);
    bool close_dead_enemy = CloseDeadEnemy(game->squareSize);
    bool attack = true;

    // game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);
    // bool f_close_enemy = CloseEnemy(0.5);
    bool f_close_max_wall = CloseMaxWall();
    bool f_bomb_on_explosion = (game->getCurrentSquare()->danger > 4);
    int number_of_bombs = game->gladiator->weapon->getBombCount();
    // bool f_time_to_explode = TimeToExplode();

    data = game->gladiator->robot->getData();
    square = game->getCurrentSquare();
    neighbors_strat[0] = square->northSquare;
    neighbors_strat[1] = square->southSquare;
    neighbors_strat[2] = square->eastSquare;
    neighbors_strat[3] = square->westSquare;
    neighbors_strat[4] = square;

    int sum = 0;
    for (int dir = 0; dir < 4; dir++)
    {
        if (neighbors_strat[dir] == nullptr) // if we have a wall
            continue;

        if (neighbors_strat[dir]->possession != '0' && neighbors_strat[dir]->possession != game->gladiator->robot->getData().teamId)
        { // if colored by the other team
            sum += 2;
        }
        if (neighbors_strat[dir]->possession == '0')
        { // if colored by the other team
            sum += 1;
        }
    }
    if (game->gladiator->maze->getCurrentMazeSize() / 0.25f < 7)
    {
        sum_threshold = 1;
    }

    switch (currentState)
    {
    case State::WAIT:
    {
        if (number_of_bombs && (sum > sum_threshold) && !(square->isBomb))
        {
            game->gladiator->weapon->dropBombs(number_of_bombs);
            sum = 0;
            square->isBomb = true;
        }

        if (f_close_max_wall || f_bomb_on_explosion)
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
            if (close_to_enemy)
            {
                if (attack)
                {
                    currentState = State::ATTACK;
                }
                else
                {
                    currentState = State::EVADE;
                }
            }
            else if(close_dead_enemy)
            {
                currentState = State::EVADE;
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
        target_square = getSafeSquare();
        game->gotoSquare(target_square);
        currentState = State::WAIT;
    }
    break;

    case State::EXPLORE:
    {
        // On cherche où sont les bombes les plus proches et on s'y dirige et on les ramasse puis explose
        current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator); // me donne les distances en mètres
        nearest_bomb = getBestBomb();
        if (nearest_bomb == nullptr)
        {
            nearest_bomb = game->gladiator->maze->getSquare(6, 6);
        }
        // game->gladiator->log("Nearest bomb %d", nearest_bomb->i);

        SimplePath path = simpleAStar(game->gladiator, current_square, nearest_bomb);
        if (path.length > 0)
        {
            if (path.length > 1)
            {
                // Move robot through path.steps[0] to path.steps[path.length-1] // Remlacer par path.length-1 pour aller direct sur les bombes (pas besoin du else dans ce cas)
                nextPos = getMazeSquareCoor(path.steps[1], game->gladiator);
                game->gotoSquare(nextPos);
            }
            else
            {
                nextPos = getMazeSquareCoor(path.steps[0], game->gladiator);
                game->gotoSquare(nextPos);
            }
        }
        else
        {
            nextPos = getMazeSquareCoor({6, 6, 0}, game->gladiator);
            game->gotoSquare(nextPos);
        }
        currentState = State::WAIT;
    }
    break;

    case State::ATTACK:
    {
        MazeSquare *current_square = getMazeSquareCoor(game->gladiator->robot->getData().position, game->gladiator);
        Position enemy_pos = locateOpponents(game->gladiator);
        MazeSquare *enemy_position = getMazeSquareCoor(enemy_pos, game->gladiator);

        SimplePath path = simpleAStar(game->gladiator, current_square, enemy_position);
        if (path.length > 0)
        {
            if (path.length > 1)
            {
                // Move robot through path.steps[0] to path.steps[path.length-1] // Remlacer par path.length-1 pour aller direct sur les bombes (pas besoin du else dans ce cas)
                nextPos = getMazeSquareCoor(path.steps[1], game->gladiator);
            }
            else
            {
                nextPos = getMazeSquareCoor(path.steps[0], game->gladiator);
            }
        }
        else
        {
            nextPos = getMazeSquareCoor({6, 6, 0}, game->gladiator);
        }
        game->gotoSquare(nextPos, 1);
        currentState = State::WAIT;
    }
    break;

    case State::EVADE:
    {
        Position enemy_pos = locateOpponents(game->gladiator);
        MazeSquare *enemy_position = getMazeSquareCoor(enemy_pos, game->gladiator);
        MazeSquare *current_square = getMazeSquareCoor(data.position, game->gladiator);
        int sg_x = 1;
        int sg_y = 1;
        if (current_square->i < enemy_position->i)
        {
            sg_x = -1;
        }
        if (current_square->j < enemy_position->j)
        {
            sg_y = -1;
        }
        MazeSquare target_square = {uint8_t(int(current_square->i) + sg_x), uint8_t(int(current_square->j) + sg_y)};
        game->gotoSquare(&target_square);
        currentState = State::WAIT;
    }
    break;
    }
}
