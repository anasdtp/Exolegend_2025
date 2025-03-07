#include <gladiator.h>
#include <cmath> // For sqrt()

// Stores the path result
struct SimplePath
{
    Position steps[144]; // Max path length for 12x12 maze
    int length = 0;
};

// Simplified A* Implementation
SimplePath simpleAStar(Gladiator *gladiator, Position start, Position target)
{
    SimplePath path;
    const float CELL_SIZE = gladiator->maze->getSquareSize();

    // Convert positions to grid coordinates
    const byte start_i = static_cast<byte>(start.x / CELL_SIZE);
    const byte start_j = static_cast<byte>(start.y / CELL_SIZE);
    const byte target_i = static_cast<byte>(target.x / CELL_SIZE);
    const byte target_j = static_cast<byte>(target.y / CELL_SIZE);

    // Early exit for invalid positions
    if (start_i >= 12 || start_j >= 12 || target_i >= 12 || target_j >= 12)
        return path;

    // Algorithm data structures
    bool visited[12][12] = {false};
    byte parent_i[12][12];
    byte parent_j[12][12];
    memset(parent_i, 0xFF, sizeof(parent_i)); // Initialize to invalid
    memset(parent_j, 0xFF, sizeof(parent_j));

    // Open list (stores nodes to explore)
    struct Node
    {
        byte i, j;
        float total_cost;
    };
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
            return path;
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
            if (!neighbor || visited[neighbor->i][neighbor->j])
                continue;

            // Calculate movement cost + heuristic
            float move_cost = current.total_cost + CELL_SIZE;
            float dx = (target_i - neighbor->i) * CELL_SIZE;
            float dy = (target_j - neighbor->j) * CELL_SIZE;
            float heuristic = sqrt(dx * dx + dy * dy);
            float total_cost = move_cost + heuristic;

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

    return path; // Empty if no path found
}