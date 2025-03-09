#include "GameData.h"

GameState::GameState(Gladiator *gladiator, Asservissement *motors)
{
    this->gladiator = gladiator;
    this->motors = motors;
    this->reset();
}

void GameState::Update()
{
    myData = gladiator->robot->getData();
    allyData = gladiator->game->getOtherRobotData(allyData.id);
    er1Data = gladiator->game->getOtherRobotData(er1Data.id);
    er2Data = gladiator->game->getOtherRobotData(er2Data.id);

    this->center_of_maze = gladiator->maze->getSquare(6, 6);
}

void GameState::reset()
{
    myData = gladiator->robot->getData();

    bool enn_init = true;
    for (int i = 0; i < 4; ++i)
    {
        byte id = gladiator->game->getPlayingRobotsId().ids[i];
        if (id == myData.id)
            continue;
        if (gladiator->game->getOtherRobotData(id).teamId == myData.teamId)
            allyData = gladiator->game->getOtherRobotData(id);
        else
        {
            if (enn_init)
            {
                er1Data = gladiator->game->getOtherRobotData(id);
                enn_init = false;
            }
            else
            {
                er2Data = gladiator->game->getOtherRobotData(id);
            }
        }
    }

    squareSize = gladiator->maze->getSquareSize();

    this->center_of_maze = gladiator->maze->getSquare(6, 6);


    switch (myData.id)
    {
    case 128:
        gladiator->robot->setCalibrationOffset(0.0054f, 0.0080f, 0.f);
        break;
    case 131:
        gladiator->robot->setCalibrationOffset(0.0009, 0.0004, 0.f);
        break;
    default:
        break;
    }
}

void GameState::gotoSquare(MazeSquare *square, int sens, float acceleration_level)
{
    goal = getSquareCoor(square, squareSize);

    motors->setAccelerationLevel(acceleration_level);
    motors->setTargetPos(goal, sens);
}

MazeSquare *GameState::getCurrentSquare(){
    return getMazeSquareCoor(gladiator->robot->getData().position, gladiator);
}

bool GameState::isOutsideArena(MazeSquare *square)
{
    int next_maze_size = int(gladiator->maze->getCurrentMazeSize() / gladiator->maze->getSquareSize());
    int min_index = (12 - next_maze_size) / 2;
    int max_index = 11 - min_index;

    if (square->i < min_index || square->i > max_index || square->j < min_index || square->j > max_index)
    {
        return true;
    }
    return false;
}