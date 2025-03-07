#ifndef ASTAR_H
#define ASTAR_H
#include "gladiator.h"
#include "Mathematiques/Mathematiques.h"
#include "GameData/GameData.h"

#define INF 255

class AStar
{
private:
    // Structures pour A*
    uint8_t gCost[SIZE][SIZE];       // Coût du chemin depuis le départ
    uint8_t fCost[SIZE][SIZE];       // f(n) = g(n) + h(n)
    MazeSquare *parents[SIZE][SIZE]; // Parent pour reconstruire le chemin

    MazeSquare *openSet[SIZE * SIZE]; // Liste des cases à explorer
    int openSetSize = 0;

    // Ajoute un élément dans l'openSet (file de priorité simplifiée)
    void addToOpenSet(MazeSquare *square);

    MazeSquare *popBestFromOpenSet();

    bool isInOpenSet(MazeSquare *square);

    // Fonction pour afficher le chemin trouvé
    void printPath(MazeSquare *start, MazeSquare *goal);

public:
    AStar(/* args */);

    void aStar(MazeSquare *start, MazeSquare *goal);

};






#endif // ASTAR_H