#include "AStar.h"

float complete_heurisic(Gladiator *gladiator, MazeSquare *neighbor) // Possession ennemi, donc id de la team, position bombe et explosion
{
    float bomb_cost = float(neighbor->danger);
    float distance_cost = 1;
    return bomb_cost + distance_cost;
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
        // Find node with lowest total cost
        int bestIndex = 0;
        for (int i = 1; i < openCount; i++)
        {
            if (openList[i].total_cost < openList[bestIndex].total_cost)
                bestIndex = i;
        }
        Node current = openList[bestIndex];

        // Path found - reconstruct
        if (current.i == target_i && current.j == target_j)
        {
            byte i = current.i, j = current.j;
            while (i != 0xFF)
            { // Follow parent pointers
                Position pos;
                pos.x = (i + 0.5f) * CELL_SIZE; // Center of cell
                pos.y = (j + 0.5f) * CELL_SIZE;
                path.steps[path.length++] = pos;
                byte pi = parent_i[i][j];
                byte pj = parent_j[i][j];
                i = pi;
                j = pj;
            }
            // Reverse to get start->target order
            for (int k = 0; k < path.length / 2; k++)
            {
                Position temp = path.steps[k];
                path.steps[k] = path.steps[path.length - 1 - k];
                path.steps[path.length - 1 - k] = temp;
            }
            return simplifyPath(path);
        }

        visited[current.i][current.j] = true;

        // Explore neighbors
        MazeSquare *square = gladiator->maze->getSquare(current.i, current.j);
        MazeSquare *neighbors[4] = {
            square->northSquare, square->southSquare,
            square->eastSquare, square->westSquare};

        for (int dir = 0; dir < 4; dir++)
        {
            MazeSquare *neighbor = neighbors[dir];
            if (!neighbor)
            {
                neighbor = gladiator->maze->getSquare(current.i, current.j + 1);
            }
            else if(visited[neighbor->i][neighbor->j])
            {
                continue;
            }

            // Calculate movement cost + heuristic
            float move_cost = current.total_cost + CELL_SIZE;
            // float complete_heurisic(Position(neighbor->i, neighbor->j), target);
            float total_cost = complete_heurisic(gladiator, neighbor) + move_cost;

            // Update existing node or add new
            bool exists = false;
            for (int i = 0; i < openCount; i++)
            {
                if (openList[i].i == neighbor->i && openList[i].j == neighbor->j)
                {
                    exists = true;
                    if (total_cost < openList[i].total_cost)
                    {
                        openList[i].total_cost = total_cost;
                        parent_i[neighbor->i][neighbor->j] = current.i;
                        parent_j[neighbor->i][neighbor->j] = current.j;
                    }
                    break;
                }
            }

            if (!exists)
            {
                openList[openCount++] = {
                    neighbor->i,
                    neighbor->j,
                    total_cost};
                parent_i[neighbor->i][neighbor->j] = current.i;
                parent_j[neighbor->i][neighbor->j] = current.j;
            }
        }
        // Remove processed node
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

    for (int i = 1; i < originalPath.length-1; i++)
    {
        Position previous = originalPath.steps[i - 1];
        Position current = originalPath.steps[i];
        Position next = originalPath.steps[i + 1];

        // Si la direction change, ajouter ce point au chemin simplifié
        if ( ! ((previous.x == current.x && current.x == next.x) || (previous.y == current.y && current.y == next.y)))
        {
            simplifiedPath.steps[simplifiedPath.length++] = current;
        }
    }

    // Ajouter le dernier point (arrivée)
    simplifiedPath.steps[simplifiedPath.length++] = originalPath.steps[originalPath.length - 1];

    return simplifiedPath;
}
