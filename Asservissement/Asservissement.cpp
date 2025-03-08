#include "Asservissement.h"

bool isAWallInFrontOfMe(Position pos, Gladiator *gladiator)
{
    MazeSquare *currentSquare = getMazeSquareCoor(pos, gladiator);

    // Déterminer la direction du déplacement
    if ((pos.a >= -PI / 4 && pos.a < PI / 4))
    { // Vers l'Est (droite)
        return currentSquare->eastSquare == NULL;
    }
    else if (pos.a >= PI / 4 && pos.a < 3 * PI / 4)
    { // Vers le Nord (haut)
        return currentSquare->northSquare == NULL;
    }
    else if ((pos.a >= 3 * PI / 4 || pos.a < -3 * PI / 4))
    { // Vers l'Ouest (gauche)
        return currentSquare->westSquare == NULL;
    }
    else
    { // Vers le Sud (bas)
        return currentSquare->southSquare == NULL;
    }
}

bool TempsEchantionnage(unsigned long TIME)
{ // ms
    static unsigned long LastMscount = millis();
    if ((millis() - LastMscount) >= TIME)
    {
        // gladiator->log("erreur temp calcul %d", (int)(millis() - LastMscount));
        LastMscount = millis();
        return true;
    }
    else
    {
        return false;
    }
}

Asservissement::Asservissement(Gladiator *gladiator)
{
    this->gladiator = gladiator;

    v_max = 1.f; acc_max = 0.8f;
    
    ta = v_max / acc_max;
    d_max = v_max * v_max / acc_max;

    goTo.Kp = 0.15f;
    goTo.Ki = 0.0f;
    goTo.Kd = 0.012f;
    goTo.integral = 0;
    goTo.prev_error = 0;

    rotation.Kp = 3.5f;
    rotation.Ki = 0.001f;
    rotation.Kd = 0.1f;
    rotation.integral = 0;
    rotation.prev_error = 0;

    Threshold = 0.05f;
    toleranceAngle = 6.f * PI / 180.f;
    consvl = 0;
    consvr = 0;

    kw = 3.f * 0.9f;
    kv = 0.75f * 0.9f;

    etat_automate_depl = INITIALISATION;
    flag_available = true;
}

Asservissement::~Asservissement()
{
}

FuncType Asservissement::fnVitesse2(Position init, Position fini)
{
    float distance = getDistance(init, fini);
    if (distance < d_max)
    {
        return [distance, this](float time)
        {
            float time_lim = sqrt(distance / acc_max);
            if (time < time_lim)
            {
                return acc_max * time;
            }
            else
            {
                return v_max - acc_max * (time - time_lim);
            }
        };
    }
    else
    {
        return [distance, this](float time)
        {
            float tc = (distance - d_max) / v_max;
            if (time < ta)
            {
                return acc_max * time;
            }
            else if (time >= ta && time <= tc + ta)
            {
                return v_max;
            }
            else
            {
                return v_max - acc_max * (time - ta - tc);
            }
        };
    }
}

float Asservissement::trajectoire(float time, FuncType velocityProfile)
{
    // Integrate velocity profile to get trajectory
    float trajectory = 0.0f;
    for (float t = 0.0f; t <= time; t += time)
    {
        trajectory += velocityProfile(t) * time;
    }

    return trajectory;
}

void Asservissement::handlePIDCoef(PIDCoef &pidGoTo, PIDCoef &pidRotation)
{
    goTo = pidGoTo;
    rotation = pidRotation;
    goTo.integral = 0;
    goTo.prev_error = 0;
    rotation.integral = 0;
    rotation.prev_error = 0;
}

// Function to calculate PID control output
float Asservissement::calculatePID(float error, float dt, PIDCoef &pid)
{
    // Variables for PID control
    pid.integral += error * dt;
    float derivative = (error - pid.prev_error) / dt;
    float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
    pid.prev_error = error;
    return output;
}

void Asservissement::positionControl(Position targetPos)
{
    currentPos = gladiator->robot->getData().position;

    switch (etat_automate_depl)
    {
        case INITIALISATION:
        {
            if (getDistance(currentPos, targetPos) > Threshold)
            {
                float dx = targetPos.x - currentPos.x;
                float dy = targetPos.y - currentPos.y;
                float angleDifference = reductionAngle(atan2(dy, dx) - currentPos.a);
                
                ta = v_max / acc_max;
                d_max = v_max * v_max / acc_max;
                traj = fnVitesse2(currentPos, targetPos);
                etat_automate_depl = GO_TO_POS;
                if (abs(angleDifference) > (PI_F / 2.f))
                {
                    sens = -1; // Reculer si l'angle est trop grand
                    angleDifference = reductionAngle(PI_F/2.f - angleDifference); // Ajustement d'angle pour la rotation en marche arrière
                }
                else
                {
                    sens = 1; // Avancer normalement
                }
        
                // // Vérifier si une rotation est nécessaire avant le déplacement
                // if (abs(angleDifference) > toleranceAngle)
                // {
                //     // Calcul de l'angle cible
                //     target_angle = angleDifference + currentPos.a;
                //     etat_automate_depl = ROTATION;
                //     Serial.println("case INITIALISATION -> ROTATION");
                // }
                // else
                // {
                //     etat_automate_depl = GO_TO_POS;
                //     Serial.println("case INITIALISATION -> GO_TO_POS");
                // }
        
                robot_radius = gladiator->robot->getRobotRadius();
                flag_available = false;
            }
            else
            {
                flag_available = true;
            }
            start_time = millis();
        }
        break;
        
    case GO_TO_POS:
    {
        dt = (millis() - start_time) * 0.001f;
        // Generate trajectory based on desired position
        float trajectory = trajectoire(dt, traj);

        float error = trajectory + getDistance(currentPos, targetPos);
        // gladiator->log("Position currentPos.x = %f, currentPos.y = %f, currentPos.a = %f, error = %f",currentPos.x, currentPos.y, currentPos.a, error);
        // gladiator->log("error = %f", error);
        float pidOutput = calculatePID(sens * error, dt, goTo);
        // // gladiator->log("pidOutput = %f", pidOutput);

        // // gladiator->log("trajectory = %f", trajectory);
        // // Calculate final motor command
        // float motor_comman d = trajectory + pidOutput;
        // gladiator->log("motor_command = %f", motor_command);

        // Calculate consvl and consvr based on go_to function logic
        float dx = targetPos.x - currentPos.x;
        float dy = targetPos.y - currentPos.y;
        float d = sqrt((dx * dx) + (dy * dy));

        if (d > Threshold)
        {
            float rho = atan2(dy, dx);
            float angleDifference = sens>0? reductionAngle(rho - currentPos.a) : reductionAngle(PI_F + rho - currentPos.a);
            float consw = kw * angleDifference;

            float consv = sens* kv * d * pow(abs(cos(abs(angleDifference))), 15);
            // consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
            // consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

            // Check if moving backward is more efficient
            
            // if (abs(angleDifference) > PI / 2.)
            // {
            //     // gladiator->log("angleDifference : %f >PI/2, dt : %f", angleDifference, dt);
            //     consv = -consv; // Move backward if turning more than 90 degrees
            // }

            consvl = consv - (robot_radius * consw); // GFA 3.6.2
            consvr = consv + (robot_radius * consw); // GFA 3.6.2
        }
        else
        {
            consvr = 0;
            consvl = 0;
        }

        if ((error < Threshold) || ((consvl + consvr) == 0))
        {
            // Serial.println("case GO_TO_POS : etat_automate_depl = ROTATION");
            // etat_automate_depl = ROTATION;
            Serial.println("case GO_TO_POS -> ARRET");
            etat_automate_depl = ARRET;
            flag_available = true;
        }

        // Apply the PID correction
        consvl += pidOutput;
        consvr += pidOutput;

        // if(isAWallInFrontOfMe(currentPos, gladiator)){
        //     consvr = 0;
        //     consvl = 0;
        //     etat_automate_depl = ROTATION;
        //     target_angle = currentPos.a + PI;
        //     Serial.println("case GO_TO_POS -> ARRET");
        // }
    }
    break;
    case ROTATION:
    {
        dt = (millis() - start_time) * 0.001f;

        // Calcul de l'erreur angulaire
        float angleError = reductionAngle(target_angle - currentPos.a);

        // Log pour le debug
        gladiator->log("currentPos.a = %f, targetAngle = %f, angleError = %f", currentPos.a*180.f/PI_F, target_angle*180.f/PI_F, angleError*180.f/PI_F);

        // Application du correcteur PID
        float pidOutput = calculatePID(angleError, dt, rotation);

        // Calcul de la commande angulaire avec PID
        // float consw = pidOutput;

        // Calcul des vitesses des roues en rotation pure
        consvl = -pidOutput * robot_radius;
        consvr = pidOutput * robot_radius;

        // Si l'erreur angulaire est suffisamment petite, on arrête la rotation
        if (abs(angleError) < toleranceAngle)
        {
            consvr = 0;
            consvl = 0;
            etat_automate_depl = INITIALISATION;
            start_time = millis();
            // Serial.println("case ROTATION -> GO_TO_POS\n");
        }
    }
    break;

    case ARRET:
    {
        Serial.println("case ARRET -> INITIALISATION\n");
        flag_available = true;
        etat_automate_depl = INITIALISATION;
        consvr = 0;
        consvl = 0;
    }
    break;
    default:
        break;
    }

    // Set wheel speeds based on the calculated motor commands
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false);
}

void Asservissement::setTargetPos(Position targetPos)
{
    this->targetPos = targetPos;
    flag_available = false;
}