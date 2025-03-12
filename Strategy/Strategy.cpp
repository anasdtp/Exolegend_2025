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

bool StateMachine::CloseEnemy(uint8_t dist_thresh)
{
    uint8_t dist1 = getDistance(game->myData.position, game->er1Data.position) / game->squareSize;
    uint8_t dist2 = getDistance(game->myData.position, game->er2Data.position) / game->squareSize;

    if (dist1 < dist_thresh && game->er1Data.lifes != 0)
    {
        if(!game->isOutsideArena(game->er1Data.position))
        {
            return true;
        }
    }

    if (dist2 < dist_thresh && game->er2Data.lifes != 0)
    {
        if(!game->isOutsideArena(game->er2Data.position))
        {
            return true;
        }
    }
    
    return false;
}

bool StateMachine::CloseDeadEnemy(uint8_t dist_thresh)
{
    uint8_t dist1 = getDistance(game->myData.position, game->er1Data.position) / game->squareSize;
    uint8_t dist2 = getDistance(game->myData.position, game->er2Data.position) / game->squareSize;

    if (dist1 < dist_thresh && game->er1Data.lifes == 0)
    {
        if(!game->isOutsideArena(game->er1Data.position))
        {
            return true;
        }
    }

    if (dist2 < dist_thresh && game->er2Data.lifes == 0)
    {
        if(!game->isOutsideArena(game->er2Data.position))
        {
            return true;
        }
    }
    
    return false;
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
        MazeSquare *square = game->getCurrentSquare();
        near = game->isOutsideFuturArena(square);
    }
    return near;
}

MazeSquare *StateMachine::getBestBomb()
{
    float max_score = -1000;
    this->current_square = game->getCurrentSquare();
    this->nearest_bomb = game->center_of_maze;

    uint8_t min_index = (SIZE - game->mazeSize) / 2; 
    uint8_t max_index = ((SIZE-1) - min_index);
    if (game->current_time % 20000 > 13000) // On se dirige vers le centre du labyrinthe si il reste 7 secondes avant le prochain retrécissement
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
                float score = -getDistance(current_square, bomb_square) + bomb_square->coin.value * 0.5f - bomb_square->danger * 1.0f - (bomb_square->possession == game->gladiator->robot->getData().teamId) * 1.0f + getDistance(bomb_square, game->center_of_maze) * 0.5f;
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
    // 1ère méthode : faire une recherche en cercle autour du robot, en ne regardant que les cases valables
    current_square = game->getCurrentSquare();
    
    for (int i = 1; i < 5; i++)
    {
        int steps = 16; // Adjust based on precision needed
        for (int k = 0; k < steps; k++)
        {
            float theta = k * (2.f * PI_F / steps);
            int dx = int(cos(theta) * i);
            int dy = int(sin(theta) * i);
            MazeSquare * square = game->gladiator->maze->getSquare(current_square->i + dx, current_square->j + dy);
            if (square != nullptr)
            {
                if((!game->isOutsideArena(square))&& square->danger <= 2){
                    // game->gladiator->log("found square with radius");
                    return square;
                }
            }
        }
    }

    if(game->isOutsideFuturArena(current_square)){
        // game->gladiator->log("isOutsideFuturArena");
        return game->center_of_maze;
    }
    return current_square;//Sinon on reste sur place en attendant un changement, comme une bombe qui explose

    // // Backup quand même ; il s'agit du code utilisé précédemment pour rentrer dans les limites du terrain
    // int sg_x = 1;
    // int sg_y = 1;
    // if (current_square->i > 6)
    // {
    //     sg_x = -1;
    // }
    // if (current_square->j > 6)
    // {
    //     sg_y = -1;
    // }

    // current_square->i = uint8_t(int(current_square->i) + sg_x);
    // current_square->j = uint8_t(int(current_square->j) + sg_y);
    // return current_square;
}

MazeSquare *StateMachine::nearestOpponent()
{
    float dist1 = getDistance(game->myData.position, game->er1Data.position);
    float dist2 = getDistance(game->myData.position, game->er2Data.position);

    if (dist1 < dist2)
    {
        return getMazeSquareCoor(game->er1Data.position, game->gladiator);
    }
    return getMazeSquareCoor(game->er2Data.position, game->gladiator);
}

void StateMachine::nearestOpponent(Position &pos)
{
    float dist1 = getDistance(game->myData.position, game->er1Data.position);
    float dist2 = getDistance(game->myData.position, game->er2Data.position);

    if (dist1 < dist2)
    {
        pos = game->er1Data.position;
    }
    else
    {
        pos = game->er2Data.position;
    }
}

void StateMachine::strategy()
{
    bool close_to_enemy = CloseEnemy(3);
    bool close_dead_enemy = CloseDeadEnemy(1);
    bool attack = true;
    
    Position enemyPos;
    nearestOpponent(enemyPos);


    float dx = enemyPos.x - game->gladiator->robot->getData().position.x;
    float dy = enemyPos.y - game->gladiator->robot->getData().position.y;
    
    float angleToEnemy = atan2(dy, dx); // Angle entre le robot et l'ennemi
    float angleDifference = reductionAngle(angleToEnemy - game->gladiator->robot->getData().position.a);

    // Déterminer le comportement en fonction de l'angle
    if (abs(angleDifference) < PI_F / 4.f) // Si l'ennemi est devant
    {
        attack = true;
    }
    else if (abs(angleDifference) > (3 * PI_F / 4.f)) // Si l'ennemi est derrière
    {
        attack = false;
    }

    // game->gladiator->log("void StateMachine::transition() : Possède une fusée : %d", t_recherche_fusee);
    // bool f_close_enemy = CloseEnemy(0.5);
    bool f_close_max_wall = CloseMaxWall();
    bool f_bomb_on_explosion = (game->getCurrentSquare()->danger > 4);
    int number_of_bombs = game->gladiator->weapon->getBombCount();
    // bool f_time_to_explode = TimeToExplode();

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
            {continue;}

        if (neighbors_strat[dir]->possession != '0' && neighbors_strat[dir]->possession != game->gladiator->robot->getData().teamId)
        { // if colored by the other team
            sum += 2;
        }
        if (neighbors_strat[dir]->possession == '0')
        { // if colored by the other team
            sum += 1;
        }
    }
    if (game->gladiator->maze->getCurrentMazeSize() / game->squareSize < 7)
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
        else //if (game->motors->available())
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
        game->gladiator->log("SURVIVAL");

        target_square = getSafeSquare();
        game->gotoSquare(target_square, 0, 1.5);
        currentState = State::WAIT;
    }
    break;
    
    case State::EXPLORE:
    {
        game->gladiator->log("EXPLORE");
        // On cherche où sont les bombes les plus proches et on s'y dirige et on les ramasse puis explose
        current_square = game->getCurrentSquare();

        nearest_bomb = getBestBomb();
        
        if (nearest_bomb == nullptr)
        {
            nearest_bomb = game->center_of_maze;
        }
        // game->gladiator->log("Nearest bomb %d", nearest_bomb->i);

        SimplePath path = simpleAStar(game, current_square, nearest_bomb);
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
            nextPos = game->center_of_maze;
        }
        game->gotoSquare(nextPos);
        currentState = State::WAIT;
    }
    break;

    case State::ATTACK:
    {
        game->gladiator->log("ATTACK");
        MazeSquare *current_square = game->getCurrentSquare();
        MazeSquare *enemy_position = nearestOpponent();

        SimplePath path = simpleAStar(game, current_square, enemy_position);
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


        if(game->isOutsideArena(nextPos)){
            currentState = State::EXPLORE;
        }
        else if(getDistance(current_square, enemy_position) < 10){
            game->motors->activateOscillationToAttack();
            currentState = State::WAIT;
        }
        else{
            game->gotoSquare(nextPos, 1);
            currentState = State::WAIT;
        }
    }
    break;

    case State::EVADE:
    {
        game->gladiator->log("EVADE");
        MazeSquare *current_square = game->getCurrentSquare();
        MazeSquare *enemy_position = nearestOpponent();
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
        MazeSquare *target_square = game->gladiator->maze->getSquare(current_square->i + sg_x, current_square->j + sg_y);
        
        if(game->isOutsideArena(target_square)){
            target_square = getSafeSquare();
        }
        else{
            game->gotoSquare(target_square);
        }
        currentState = State::WAIT;
    }
    break;
    }
}