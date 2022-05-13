#ifndef __PISTON__
#define __PISTON__

#include "main.h"


class Piston
{
private:
    pros::ADIDigitalOut piston;
    bool status;
    int num_actuations;

public:
    Piston(char port, bool init_status=false) 
    : piston(port) {
        set(init_status);
    }

    void set(bool extend)
    {
        piston.set(extend);
        status = extend;
    }

    void toggle()
    {
        set(!status);
    }

    bool status()
    {
        return status;
    }
};

#endif