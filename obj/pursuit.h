#ifndef __PURSUIT__
#define __PURSUIT__

#include "main.h"
#include "chassis.h"
#include "pid.h"
#include <math.h>


#define PI 3.14159265

class PurePursuit
{
private:
    Chassis chas;
    PID driveK;
    PID correctionK;
    pros::Imu imu;
    pros::ADIEncoder odom;
    
    double x_pos, y_pos;
    double degree;

public:
    PurePursuit(Chassis init_chas, pros::Imu init_imu, pros::ADIEncoder init_odom, PID init_driveK=PID(0.5,0,0), PID init_correctionK=PID(1,0,0))
    : chas(init_chas), imu(init_imu), odom(init_odom), driveK(init_driveK), correctionK(init_correctionK) {
        imu.set_heading(180);
        x_pos = 0;
        y_pos = 0;
        degree = 0;
    }

    void reset()
    {
        odom.reset();
        imu.set_heading(180);
        degree = 0;
        chas.reset();
    }

    void drive_to(double x_tar, double y_tar, double deg_tar, float speed=1.0, int timeout=5000)
    {
        double x_err = x_tar - x_pos;
        double y_err = y_tar - y_pos;
        double magnitude = sqrt(x_err * x_err + y_err * y_err);
        double odom_val = odom.get_value();
        double last_odom_val = odom.get_value();

        degree = fmod(degree, 360);
        imu.set_heading(180);
        double target_heading = 180 + deg_tar - degree;
        double deg_err;
        target_heading = (target_heading > 180) ? -(360 - target_heading) : ((target_heading < -180) ? (360 + target_heading) : (target_heading));

        int time = 0;

        while(time < timeout)
        {
            odom_val = odom.get_value();
            x_pos += (odom_val - last_odom_val) * sin((180 - imu.get_heading()) * PI / 180);
            y_pos += (odom_val - last_odom_val) * cos((180 - imu.get_heading()) * PI / 180);
            last_odom_val = odom_val;

            x_err = x_tar - x_pos;
            y_err = y_tar - y_pos;
            magnitude = sqrt(x_err * x_err + y_err * y_err);
            deg_err = target_heading - imu.get_heading();

            chas.spin_left(std::min(driveK.calculate(magnitude), 127) + correctionK.calculate(deg_err));
            chas.spin_right(std::min(driveK.calculate(magnitude), 127) - correctionK.calculate(deg_err));

            pros::delay(1);
            time += 1;
        }
        chas.stop();
    }
};

#endifpros