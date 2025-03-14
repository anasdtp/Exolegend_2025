#include "AStar.h"
float WALL_WEIGHT = 10.0f;

MazeSquare *square;
MazeSquare *square_heuresique;
MazeSquare *neighbors[4];
MazeSquare *neighbor;
MazeSquare *robot_square;

float heuristic_rotation(GameState *game, uint8_t ni, uint8_t nj)
{
    Position current_pos = game->myData.position;
    robot_square = game->getCurrentSquare();

    float angle_to_turn = 0;
    uint8_t i_n = ni;
    uint8_t j_n = nj;
    uint8_t i_r = robot_square->i;
    uint8_t j_r = robot_square->j;
    float round_current_orientation;
    if (current_pos.a < PI / 4 && current_pos.a > -PI / 4)
    {
        round_current_orientation = 0;
    }
    else if (current_pos.a < 3 * PI / 4 && current_pos.a > PI / 4)
    {
        round_current_orientation = PI / 2;
    }
    else if (current_pos.a < -3 * PI / 4 && current_pos.a > 3 * PI / 4)
    {
        round_current_orientation = -PI;
    }
    else if (current_pos.a < -PI / 4 && current_pos.a > -3 * PI / 4)
    {
        round_current_orientation = -PI / 2;
    }

    if (i_n == i_r)
    {
        if (j_n - j_r == 1)
        {
            angle_to_turn = -(round_current_orientation - 0);
        }
        else if (j_n - j_r == -1)
        {
            angle_to_turn = -(round_current_orientation - PI);
        }
    }
    else if (j_n == j_r)
    {
        if (i_n - i_r == 1)
        {
            angle_to_turn = -(round_current_orientation - (-PI / 2));
        }
        else if (i_n - i_r == -1)
        {
            angle_to_turn = -(round_current_orientation - (PI / 2));
        }
    }
    float angle_cost{0};
    if (angle_to_turn == PI / 2 || angle_to_turn == -PI / 2)
    {
        angle_cost = 10;
    }
    else if (angle_to_turn == PI)
    {
        angle_cost = 20;
    }
    return angle_cost;
}

float complete_heurisic(GameState *game, uint8_t ni, uint8_t nj)
{
    uint8_t min_index = (SIZE - game->mazeSize) / 2; 
    uint8_t max_index = ((SIZE - 1) - min_index);

    float cost_outside_square = 0;
    if (ni < min_index || ni > max_index || nj < min_index || nj > max_index)
    {
        cost_outside_square = 10000.0f;
    }

    float cost_lives = 0.f;
    if (!game->er1Data.lifes || !game->er2Data.lifes)
    {
        cost_lives = 500.f;
    }

    square_heuresique = game->gladiator->maze->getSquare(ni, nj);
    float bomb_cost = (square_heuresique != nullptr) ? 2 * square_heuresique->danger : 0;
    float angle_cost = heuristic_rotation(game, ni, nj);
    return bomb_cost + angle_cost + 1.0f + cost_outside_square + cost_lives; // +1 for distance
}

// Simplified A* Implementation
SimplePath simpleAStar(GameState *game, MazeSquare *current_square, MazeSquare *target_square)
{
    SimplePath path;
    path.length = 0;
    
    // Early exit for invalid positions
    if (game->isOutsideArena(current_square) || game->isOutsideArena(target_square))
    {return path;}

    square = nullptr;
    neighbor = nullptr;
    neighbors[0] = nullptr;
    neighbors[1] = nullptr;
    neighbors[2] = nullptr;
    neighbors[3] = nullptr;

    // Position start = getSquareCoor(current_square, game->squareSize);
    // Position target = getSquareCoor(target_square, game->squareSize);

    // Algorithm data structures
    bool visited[12][12] = {false};
    uint8_t parent_i[12][12];
    uint8_t parent_j[12][12];
    memset(parent_i, 0xFF, sizeof(parent_i)); // Initialize to invalid
    memset(parent_j, 0xFF, sizeof(parent_j));

    Node openList[144];
    int openCount = 0;

    // Start with initial position
    openList[openCount++] = {current_square->i, current_square->j, 0};

    while (openCount > 0)
    {
        // Trouver le nœud avec le coût total le plus bas
        int bestIndex = 0;
        for (int i = 1; i < openCount; i++)
        {
            if (openList[i].total_cost < openList[bestIndex].total_cost)
                {bestIndex = i;}
        }
        Node current = openList[bestIndex];

        // Chemin trouvé - reconstruction
        if (current.i == target_square->i && current.j == target_square->j)
        {
            uint8_t i = current.i, j = current.j;
            while (i != 0xFF)
            { // Suivre les parents
                path.steps[path.length++] = {
                    (i + 0.5f) * game->squareSize,
                    (j + 0.5f) * game->squareSize};
                uint8_t pi = parent_i[i][j], pj = parent_j[i][j];
                i = pi;
                j = pj;
            }
            // Inversion pour obtenir l'ordre start → target
            for (int k = 0; k < path.length / 2; k++)
            {
                Position temp = path.steps[k];
                path.steps[k] = path.steps[path.length - 1 - k];
                path.steps[path.length - 1 - k] = temp;
            }
            return simplifyPath(path);
        }

        visited[current.i][current.j] = true;

        // Exploration des voisins directement sans tableau temporaire
        static const int dx[4] = {0, 0, 1, -1};
        static const int dy[4] = {1, -1, 0, 0};

        square = game->gladiator->maze->getSquare(current.i, current.j);
        neighbors[0] = square->northSquare;
        neighbors[1] = square->southSquare;
        neighbors[2] = square->eastSquare;
        neighbors[3] = square->westSquare;

        for (int dir = 0; dir < 4; dir++)
        {
            // Inside the neighbor loop:
            uint8_t ni = current.i + dx[dir];
            uint8_t nj = current.j + dy[dir];

            // Skip out-of-bounds or visited nodes
            if (ni >= 12 || nj >= 12 || visited[ni][nj])
                continue;

            neighbor = neighbors[dir];
            bool isWall = (neighbor == nullptr);

            float move_cost = current.total_cost + game->squareSize;
            if (isWall)
            {
                move_cost += WALL_WEIGHT; // Apply penalty
                neighbor = game->gladiator->maze->getSquare(ni, nj);
            }

            float heuristic = complete_heurisic(game, ni, nj);
            float total_cost = move_cost + heuristic;

            // Update or add to open list
            bool exists = false;
            for (int i = 0; i < openCount; i++)
            {
                if (openList[i].i == ni && openList[i].j == nj)
                {
                    exists = true;
                    if (total_cost < openList[i].total_cost)
                    {
                        openList[i].total_cost = total_cost;
                        parent_i[ni][nj] = current.i;
                        parent_j[ni][nj] = current.j;
                    }
                    break;
                }
            }

            if (!exists && openCount < 144)
            {
                openList[openCount++] = {ni, nj, total_cost};
                parent_i[ni][nj] = current.i;
                parent_j[ni][nj] = current.j;
            }
        }

        // Supprimer le nœud traité
        openList[bestIndex] = openList[--openCount];
    }

    return simplifyPath(path); // Empty if no path found
}

SimplePath simplifyPath(const SimplePath &originalPath)
{
    SimplePath simplifiedPath;
    if (originalPath.length == 0)
        return simplifiedPath; // Retourne un chemin vide si aucun chemin n'a été trouvé

    // Ajouter le premier point (départ)
    simplifiedPath.steps[simplifiedPath.length++] = originalPath.steps[0];

    for (int i = 1; i < originalPath.length - 1; i++)
    {
        Position previous = originalPath.steps[i - 1];
        Position current = originalPath.steps[i];
        Position next = originalPath.steps[i + 1];

        // Si la direction change, ajouter ce point au chemin simplifié
        if (!((previous.x == current.x && current.x == next.x) || (previous.y == current.y && current.y == next.y)))
        {
            simplifiedPath.steps[simplifiedPath.length++] = current;
        }
    }

    // Ajouter le dernier point (arrivée)
    simplifiedPath.steps[simplifiedPath.length++] = originalPath.steps[originalPath.length - 1];

    return simplifiedPath;
}
