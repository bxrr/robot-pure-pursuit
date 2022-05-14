#ifndef __AUT_FUNC__
#define __AUT_FUNC__

#include "main.h"
#include "chassis.h"
#include "pid.h"


class AutonFunc
{
private:
    Chassis chas;
    PID driveK;
    PID rotateK;
    double degree;

public:
    AutonFunc(Chassis init_chas, PID init_driveK=PID(1, 0, 0), PID init_rotateK=PID(1, 0, 0))
    : chas(chas), driveK(init_driveK), rotateK(init_rotateK) {
        degree = 0;
    }
};

#endif