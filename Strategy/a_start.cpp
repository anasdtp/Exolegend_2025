#include "a_star.h"
#include <gladiator.h>
#include <cmath>

// Simplified path container
struct Path
{
    Position positions[144]; // Max 12x12 nodes
    int size = 0;
};

// A* node structure
// struct Node
// {
//     byte i, j;
//     float g, h, f;
//     byte parent_i, parent_j;
//     bool operator<(const Node &other) const { return f < other.f; }
// };

// A* implementation
Path aStarPathfinding(Gladiator *gladiator, Position start, Position target)
{
    Path path;
    const float squareSize = gladiator->maze->getSquareSize();

    // Convert positions to grid coordinates
    const byte start_i = static_cast<byte>(start.x / squareSize);
    const byte start_j = static_cast<byte>(start.y / squareSize);
    const byte target_i = static_cast<byte>(target.x / squareSize);
    const byte target_j = static_cast<byte>(target.y / squareSize);

    // Sanity checks
    if (start_i >= 12 || start_j >= 12 || target_i >= 12 || target_j >= 12)
        return path;

    // Track visited nodes (closed list)
    bool closedList[12][12] = {false};

    // Parent tracking
    byte parent_i[12][12];
    byte parent_j[12][12];
    memset(parent_i, 0xFF, sizeof(parent_i)); // Initialize to invalid
    memset(parent_j, 0xFF, sizeof(parent_j));

    // Open list (priority queue simulation)
    Node openList[144];
    int openSize = 0;

    // Add start node
    openList[openSize++] = {
        start_i, start_j,
        0,
        heuristic(start_i, start_j, target_i, target_j, squareSize),
        0 + heuristic(start_i, start_j, target_i, target_j, squareSize),
        0xFF, 0xFF // No parent
    };

    while (openSize > 0)
    {
        // Find node with lowest f in open list
        int bestIndex = 0;
        for (int i = 1; i < openSize; i++)
        {
            if (openList[i].f < openList[bestIndex].f)
                bestIndex = i;
        }
        Node current = openList[bestIndex];

        // Remove from open list
        openList[bestIndex] = openList[--openSize];

        // Check if target reached
        if (current.i == target_i && current.j == target_j)
        {
            // Reconstruct path
            byte i = current.i, j = current.j;
            while (i != 0xFF && j != 0xFF)
            {
                Position pos;
                pos.x = i * squareSize + squareSize / 2;
                pos.y = j * squareSize + squareSize / 2;
                path.positions[path.size++] = pos;

                byte pi = parent_i[i][j];
                byte pj = parent_j[i][j];
                i = pi;
                j = pj;
            }
            // Reverse path to get start->target order
            for (int k = 0; k < path.size / 2; k++)
            {
                Position temp = path.positions[k];
                path.positions[k] = path.positions[path.size - 1 - k];
                path.positions[path.size - 1 - k] = temp;
            }
            return path;
        }

        closedList[current.i][current.j] = true;

        // Get neighbors
        MazeSquare *currentSquare = gladiator->maze->getSquare(current.i, current.j);
        MazeSquare *neighbors[4] = {
            currentSquare->northSquare,
            currentSquare->southSquare,
            currentSquare->eastSquare,
            currentSquare->westSquare};

        for (int dir = 0; dir < 4; dir++)
        {
            MazeSquare *neighbor = neighbors[dir];
            if (!neighbor)
                continue;

            const byte ni = neighbor->i;
            const byte nj = neighbor->j;

            if (closedList[ni][nj])
                continue;

            const float tentative_g = current.g + squareSize;

            // Check if already in open list
            bool inOpen = false;
            for (int i = 0; i < openSize; i++)
            {
                if (openList[i].i == ni && openList[i].j == nj)
                {
                    inOpen = true;
                    if (tentative_g < openList[i].g)
                    {
                        openList[i].g = tentative_g;
                        openList[i].f = openList[i].g + openList[i].h;
                        parent_i[ni][nj] = current.i;
                        parent_j[ni][nj] = current.j;
                    }
                    break;
                }
            }

            if (!inOpen)
            {
                openList[openSize++] = {
                    ni, nj,
                    tentative_g,
                    heuristic(ni, nj, target_i, target_j, squareSize),
                    tentative_g + heuristic(ni, nj, target_i, target_j, squareSize),
                    current.i, current.j};
                parent_i[ni][nj] = current.i;
                parent_j[ni][nj] = current.j;
            }
        }
    }

    return path; // Empty if no path
}

// Heuristic helper
float heuristic(byte i1, byte j1, byte i2, byte j2, float squareSize)
{
    float dx = (i1 - i2) * squareSize;
    float dy = (j1 - j2) * squareSize;
    return sqrt(dx * dx + dy * dy);
}