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

void GameState::gotoSquare(MazeSquare *square, int sens)
{
    goal = getSquareCoor(square, squareSize);

    motors->setTargetPos(goal, sens);
}

MazeSquare *GameState::getCurrentSquare(){
    return getMazeSquareCoor(gladiator->robot->getData().position, gladiator);
}