#ifndef __DRIVER__
#define __DRIVER__

#include "main.h"
#include "global.h"

using namespace glb;
using namespace pros;


void arcade_drive()
{
    double left = abs(con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) > 10 ? con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) : 0;
    double right = abs(con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)) > 10 ? con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) : 0;

    if(left || right)
    {
        chas.spin_left(left + right);
        chas.spin_right(left - right);
    }

    else
    {
        chas.stop();
    }
}

void print_info(int time)
{
    if(time % 500 == 0 && time % 200 != 0 && time % 200 != 0) con.print(0, 0, "TEMP: %.1lf         ", chas.temp());
    if(time % 200 == 0 && time % 500 != 0 && time % 5000 != 0) con.print(1, 0, "%.2f : %.2f", imu.get_heading(), chas.pos());
}

void calibrate_robot()
{
    imu.reset();
    chas.reset();
    aut_controller.reset();
}

#endif 