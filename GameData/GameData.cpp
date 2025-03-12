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

    squareSize = gladiator->maze->getSquareSize();
    mazeSize = uint8_t(round(gladiator->maze->getCurrentMazeSize() / squareSize));
    center_of_maze = gladiator->maze->getSquare(SIZE / 2, SIZE / 2);

    min_index = (SIZE - this->mazeSize) / 2; 
    max_index = ((SIZE - 1) - min_index);

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

    center_of_maze = gladiator->maze->getSquare(SIZE / 2, SIZE / 2);

    mazeSize = uint8_t(round(gladiator->maze->getCurrentMazeSize() / squareSize));

    min_index = (SIZE - this->mazeSize) / 2; 
    max_index = ((SIZE - 1) - min_index);


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

bool GameState::isOutsideArena(MazeSquare *square)
{
    if (square->i < min_index || square->i > max_index || square->j < min_index || square->j > max_index)
    {
        return true;
    }
    return false;
}

bool GameState::isOutsideArena(Position pos)
{
    return isOutsideArena(getMazeSquareCoor(pos, gladiator));
}

bool GameState::isOutsideFuturArena(MazeSquare *square)
{
    if (square->i < (min_index + 1) || square->i > (max_index - 1) || square->j < (min_index + 1) || square->j > (max_index - 1))
    {
        return true;
    }
    return false;
}