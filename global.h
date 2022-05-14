#ifndef __GLOBAL__
#define __GLOBAL__

#include "main.h"
#include "obj/chassis.h"
#include "obj/pid.h"
#include "obj/piston.h"
#include "obj/pursuit.h"

namespace glb
{
    // ports ===============================
    #define P_IMU 1
    #define P_BL 2
    #define P_FL 3
    #define P_BR 4
    #define P_FR 5

    #define P_ODOM_TOP 'A'
    #define P_ODOM_BOT 'B'

    // objects =============================
    pros::Controller con(pros::E_CONTROLLER_MASTER);
    pros::Imu imu(P_IMU);
    pros::ADIEncoder odom(P_ODOM_TOP, P_ODOM_BOT);
    Chassis chas({P_BL, P_FL}, {P_BR, P_FR}, pros::E_MOTOR_GEARSET_06, false);
    
    PurePursuit aut_controller(chas, imu, odom, PID(1,0,0));
}

#endif