#ifndef ASSERVISSEMENT_H
#define ASSERVISSEMENT_H
#include "gladiator.h"
#include "Mathematiques/Mathematiques.h"
#include <functional>

using FuncType = std::function<float(float)>;

#include <functional>

#define TE_MS 50
#define TE (TE_MS * 0.001)

#define INITIALISATION 0
#define GO_TO_POS 1
#define ATTAQUE 2
#define ROTATION 4
#define ARRET 3

typedef struct
{
    float Kp; // = 0.04f; // Proportional gain
    float Ki; // = 0.f;    // Integral gain
    float Kd; // = 0.1f; // Derivative gain

    float integral, prev_error;
} PIDCoef;

bool TempsEchantionnage(unsigned long TIME);

class Asservissement
{
private:
    Position currentPos;
    Position targetPos;

    float acc_max; // = 0.6f;
    float v_max;   // = 0.8f;
    float ta;
    float d_max;

    FuncType traj;

    PIDCoef goTo;
    PIDCoef rotation;

    float robot_radius;

    float Threshold;
    float toleranceAngle; // = 8.f * PI/180.f; // Adjust as needed
    float consvl, consvr;

    float kw; // = 3.f * 2.f;
    float kv; // = 0.75f * 2.f;

    unsigned long start_time;
    float dt;

    float sens;
    float forcer_sens;
    float target_angle; //Pour l'etat ROTATION

    float sens_rotation_attaque;

    int etat_automate_depl = INITIALISATION;
    int next_state = INITIALISATION;//Pour que l'etat ROTATION sache où il doit aller après avoir fini de tourner

    bool flag_available;

    Gladiator *gladiator;

    FuncType fnVitesse2(Position init, Position fini);
    float trajectoire(float time, FuncType velocityProfile);

    float calculatePID(float error, float dt, PIDCoef &pid);

public:
    Asservissement(Gladiator *gladiator);
    ~Asservissement();

    void handlePIDCoef(PIDCoef &pidGoTo, PIDCoef &pidRotation);

    void positionControl(Position targetPos);

    /*
    * @brief Fonction pour déplacer le robot vers une position donnée
    * @param targetPos : la position vers laquelle on veut déplacer le robot en mètres
    * @param sens : le sens de déplacement
    *             (0 pour laisser le robot choisir le sens optimal,
    *             1 pour forcer le déplacement en avant,
    *            -1 pour forcer le déplacement en arrière)
    */
    void setTargetPos(Position targetPos, int sens = 0);

    Position getTargetPos()
    {
        return targetPos;
    }

    bool available()
    {
        return flag_available;
    }

    void activateOscillationToAttack();

};

#endif // ASSERVISSEMENT_H