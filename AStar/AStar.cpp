#include "AStar.h"

AStar::AStar(/* args */)
{
}

// Ajoute un élément dans l'openSet (file de priorité simplifiée)
void AStar::addToOpenSet(MazeSquare *square) {
    openSet[openSetSize++] = square;
}

// Retire le meilleur élément (plus petit fCost)
MazeSquare *AStar::popBestFromOpenSet() {
    int bestIndex = 0;
    for (int i = 1; i < openSetSize; i++) {
        if (fCost[openSet[i]->i][openSet[i]->j] < fCost[openSet[bestIndex]->i][openSet[bestIndex]->j]) {
            bestIndex = i;
        }
    }
    
    MazeSquare *bestSquare = openSet[bestIndex];

    // Supprimer l'élément
    openSetSize--;
    for (int i = bestIndex; i < openSetSize; i++) {
        openSet[i] = openSet[i+1];
    }
    
    return bestSquare;
}

// Vérifie si une case est dans l'openSet
bool AStar::isInOpenSet(MazeSquare *square) {
    for (int i = 0; i < openSetSize; i++) {
        if (openSet[i] == square)
            return true;
    }
    return false;
}

// Trouve un chemin entre start et goal
void AStar::aStar(MazeSquare *start, MazeSquare *goal) {
    // Initialisation des coûts
    for (int i = 0; i < SIZE; i++)
        for (int j = 0; j < SIZE; j++)
            gCost[i][j] = fCost[i][j] = INF;
    
    gCost[start->i][start->j] = 0;
    fCost[start->i][start->j] = heuristic(start, goal);
    openSetSize = 0;
    addToOpenSet(start);

    while (openSetSize > 0) {
        MazeSquare *current = popBestFromOpenSet();

        // Si on atteint la destination, on sort
        if (current == goal) {
            printf("Chemin trouvé !\n");
            return;
        }

        // Explorer les voisins valides
        MazeSquare *neighbors[4] = {current->northSquare, current->southSquare, current->westSquare, current->eastSquare};

        for (int d = 0; d < 4; d++) {
            MazeSquare *neighbor = neighbors[d];
            if (neighbor == NULL) continue;  // Mur ou hors limites
            
            uint8_t tentativeGCost = gCost[current->i][current->j] + 1;
            if (tentativeGCost < gCost[neighbor->i][neighbor->j]) {
                gCost[neighbor->i][neighbor->j] = tentativeGCost;
                fCost[neighbor->i][neighbor->j] = tentativeGCost + heuristic(neighbor, goal);
                parents[neighbor->i][neighbor->j] = current;
                
                if (!isInOpenSet(neighbor)) {
                    addToOpenSet(neighbor);
                }
            }
        }
    }
    
    printf("Aucun chemin trouvé.\n");
}

// Fonction pour afficher le chemin trouvé
void AStar::printPath(MazeSquare *start, MazeSquare *goal) {
    MazeSquare *current = goal;
    while (current != start) {
        printf("(%d, %d) <- ", current->i, current->j);
        current = parents[current->i][current->j];
    }
    printf("(%d, %d)\n", start->i, start->j);
}