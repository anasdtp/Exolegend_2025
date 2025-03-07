#include "gladiator.h"
#include "GameData/GameData.h"
#include "Mathematiques/Mathematiques.h"
// AStar.h
#pragma once
#include <cmath>
#include <cstring> // For memset

// Simplified path container
struct Path
{
    Position positions[144]; // Max 12x12 nodes
    int size = 0;
};

// A* node structure
struct Node
{
    byte i, j;
    float g, h, f;
    byte parent_i, parent_j;
    bool operator<(const Node &other) const { return f < other.f; }
};

// Helper function declarations
inline float heuristic(byte i1, byte j1, byte i2, byte j2, float squareSize);
// inline Path aStarPathfinding(Gladiator* gladiator, Position start, Position target);

// ========== Implementation ==========
inline float heuristic(byte i1, byte j1, byte i2, byte j2, float squareSize)
{
    float dx = (i1 - i2) * squareSize;
    float dy = (j1 - j2) * squareSize;
    return std::sqrt(dx * dx + dy * dy);
}

inline Path aStarPathfinding(Gladiator *gladiator, Position start, Position target)
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
    memset(parent_i, 0xFF, sizeof(parent_i));
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

    // ... [Rest of the A* implementation from previous answer] ...

    return path;
}