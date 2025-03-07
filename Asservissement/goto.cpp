#include "goto.h"

using namespace std;

bool followPath(GameState *game)
{
    static int count_lecture = 0;
    // Position current = game->gladiator->robot->getData().position; // game->myData.position;

    // go_to(game->goal, current, game->gladiator);

    bool occupation = ((game->count - count_lecture) != 0);

    // if (getDistance(current, game->goal) <= THRESHOLD && occupation)
    if(occupation && (game->motors->available()))
    {
        game->goal = getSquareCoor(game->simplified_coord_list.path_coord[count_lecture].i, game->simplified_coord_list.path_coord[count_lecture].j, game->squareSize);
        game->gladiator->log("i: %d | j: %d", game->simplified_coord_list.path_coord[count_lecture].i, game->simplified_coord_list.path_coord[count_lecture].j);
        count_lecture = (count_lecture + 1) % max_parth_finder_size;

        game->motors->setTargetPos(game->goal);

        game->gladiator->log("followPath(GameState *game) : Position game->goal.x = %f, game->goal.y = %f, game->goal.a = %f", game->goal.x, game->goal.y, game->goal.a);
        return true;
    }
    else if(occupation){
        return true;
    }
    return false;
}