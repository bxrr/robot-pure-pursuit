#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#include "main.h"
#include "chassis.h"
#include "pid.h"
#include <math.h>


#define PI 3.14159265

class PurePursuit
{
private:
    Chassis chas;
    PID k;
    pros::Imu imu;
    pros::ADIEncoder odom;
    
    double x_pos, y_pos;
    double degree;

public:
    MotionProfiler(Chassis init_chas, PID k, pros::Imu init_imu, pros::ADIEncoder init_odom)
    : chas(init_chas), k(init_k), odom(init_odom), xpos(0), ypos(0), heading(0) {
        imu.set_heading(180);
    }

    void zero_position()
    {
        odom.reset();
        imu.set_heading(180);
        degree = 0;
    }

    void drive_to(double x_tar, double y_tar, double deg_tar, float speed=1.0, int timeout=5000)
    {
        double x_err = x_tar - x_pos;
        double y_err = y_tar - y_pos;
        double magnitude = sqrt(x_err * x_err + y_err * y_err);
        double odom_val = odom.get_value();
        double last_odom_val = odom.get_value();

        degree %= 360;
        imu.set_heading(180);
        double turn_amount = deg_tar - degree;
        double deg_err;
        turn_amount = (turn_amount > 180) ? -(360 - turn_amount) : ((turn_amount < -180) ? (360 + turn_amount) : (turn_amount)); // optimize turn direction

        int time = 0;

        while(time < timeout)
        {
            odom_val = odom.get_value();
            x_pos += (odom_val - last_odom_val) * sin((180 - imu.get_heading()) * PI / 180);
            y_pos += (odom_val - last_odom_val) * cos((180 - imu.get_heading()) * PI / 180);
            last_odom_val = odom_val

            x_err = x_tar - x_pos;
            y_err = y_tar - y_pos;
            magnitude = sqrt(x_err * x_err + y_err * y_err);
            deg_err = (180 + turn_amount) - imu.get_heading();

            double left_mult = (x_err + deg_err) / (y_err >= 1 ? y_err : 1);
            double right_mult = (y_err - deg_err) / (x_err >= 1 ? x_err : 1);
            if(max(left_mult, right_mult) * magnitute > 127)
                magnitute = 127 / max(left_mult, right_mult);

            double left_speed = magnitute * left_mult;
            double right_speed = magnitute * right_mult;

            chas.spin_left(k.kP * left_speed + k.kD * last_odom_val - odom_val * speed);
            chas.spin_right(k.kP * right_speed + k.kD * last_odom_val - odom_val * speed);

            pros::delay(10);
            time += 10;
        }
        chas.stop();
    }
};

#endif