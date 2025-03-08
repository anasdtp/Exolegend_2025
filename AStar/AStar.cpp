#include "AStar.h"
float WALL_WEIGHT = 10.0f;

float heuristic_rotation(Gladiator *gladiator, byte ni, byte nj)
{
    Position current_pos = gladiator->robot->getData().position;
    MazeSquare *robot_square = getMazeSquareCoor(current_pos, gladiator);

    float angle_to_turn;
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

float complete_heurisic(Gladiator *gladiator, byte ni, byte nj)
{
    MazeSquare *square = gladiator->maze->getSquare(ni, nj);
    float bomb_cost = (square != nullptr) ? 2 * square->danger : 0;
    float angle_cost = heuristic_rotation(gladiator, ni, nj);
    return bomb_cost + angle_cost + 1; // +1 for distance
}

// Simplified A* Implementation
SimplePath simpleAStar(Gladiator *gladiator, MazeSquare *current_square, MazeSquare *target_square)
{
    SimplePath path;
    path.length = 0;
    float CELL_SIZE = gladiator->maze->getSquareSize();

    Position start = getSquareCoor(current_square, CELL_SIZE);
    Position target = getSquareCoor(target_square, CELL_SIZE);

    // Convert positions to grid coordinates
    byte start_i = static_cast<byte>(start.x / CELL_SIZE);
    byte start_j = static_cast<byte>(start.y / CELL_SIZE);
    byte target_i = static_cast<byte>(target.x / CELL_SIZE);
    byte target_j = static_cast<byte>(target.y / CELL_SIZE);

    // Early exit for invalid positions
    if (start_i >= 12 || start_j >= 12 || target_i >= 12 || target_j >= 12)
        return path;

    // Algorithm data structures
    bool visited[12][12] = {false};
    byte parent_i[12][12];
    byte parent_j[12][12];
    memset(parent_i, 0xFF, sizeof(parent_i)); // Initialize to invalid
    memset(parent_j, 0xFF, sizeof(parent_j));

    Node openList[144];
    int openCount = 0;

    // Start with initial position
    openList[openCount++] = {start_i, start_j, 0};

    while (openCount > 0)
    {
        // Trouver le nœud avec le coût total le plus bas
        int bestIndex = 0;
        for (int i = 1; i < openCount; i++)
        {
            if (openList[i].total_cost < openList[bestIndex].total_cost)
                bestIndex = i;
        }
        Node current = openList[bestIndex];

        // Chemin trouvé - reconstruction
        if (current.i == target_i && current.j == target_j)
        {
            byte i = current.i, j = current.j;
            while (i != 0xFF)
            { // Suivre les parents
                path.steps[path.length++] = {
                    (i + 0.5f) * CELL_SIZE,
                    (j + 0.5f) * CELL_SIZE};
                byte pi = parent_i[i][j], pj = parent_j[i][j];
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

        MazeSquare *square = gladiator->maze->getSquare(current.i, current.j);
        MazeSquare *neighbors[4] = {
            square->northSquare, square->southSquare,
            square->eastSquare, square->westSquare};

        for (int dir = 0; dir < 4; dir++)
        {
            // Inside the neighbor loop:
            byte ni = current.i + dx[dir];
            byte nj = current.j + dy[dir];

            // Skip out-of-bounds or visited nodes
            if (ni >= 12 || nj >= 12 || visited[ni][nj])
                continue;

            MazeSquare *neighbor = neighbors[dir];
            bool isWall = (neighbor == nullptr);

            float move_cost = current.total_cost + CELL_SIZE;
            if (isWall)
            {
                move_cost += WALL_WEIGHT; // Apply penalty
                neighbor = gladiator->maze->getSquare(ni, nj);
            }

            float heuristic = complete_heurisic(gladiator, ni, nj);
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
