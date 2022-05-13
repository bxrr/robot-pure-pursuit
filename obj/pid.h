#ifndef __PID__
#define __PID__

#include "main.h"


class PID
{
private:
    double integral;

public:
    const double kP, kI, kD;

    PID(double init_kP, double init_kI=0, double init_kD=0)
    : kP(init_kP), kI(init_kI), kD(init_kD), integral(0)
    {}

    double calculate(double error, double multiplier=1.0, bool count_i)
    {
        integral += error * count_i;
        static double last_error = error;
        return multiplier * (error * kP + integral * kI + (error - last_error) * kD);
    }
    
    void reset_i()
    {
        integral = 0;
    }
};

#endif