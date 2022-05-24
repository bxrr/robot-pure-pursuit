#ifndef __ODOM_CONTROLLER__
#define __ODOM_CONTROLLER__

#include <math.h>
#include "main.h"
#include "chassis.h"
#include "pid.h"
#include "aut_controller.h"

#define PI 3.14159265

class OdomController : public AutonController
{
private:
    double x, y;
    pros::ADIEncoder odom;
    double last_odom_val;

public:
    OdomController(Chassis init_chas, pros::Imu init_imu, pros::ADIEncoder init_odom, PID init_driveK=PID(1, 0, 0), PID init_rotateK=PID(1, 0, 0), PID init_straightK=PID(0, 0, 0))
    : odom(init_odom), AutonController(init_chas, init_imu, init_driveK, init_rotateK, init_straightK) {
        x = 0;
        y = 0;
        last_odom_val = 0;
    }

    void drive_to(double x_tar, double y_tar)
    {
        double x_err = x_tar - x;
        double y_err = y_tar - y;
        rotate_to(atan2(y_err, x_err) * 180 / PI);

        x_err = x_tar - x;
        y_err = y_tar - y;
        drive(sqrt(x_err * x_err + y_err * y_err));
    }

    void drive(double distance, int timeout=5000, double max_speed=120, pros::Controller con=pros::Controller(pros::E_CONTROLLER_MASTER))
    {
        int time = 0;
        
        last_odom_val = odom.get_value();
        double target = distance + odom.get_value();

        imu.set_heading(180);
        double target_heading = 180 + (ideal_degree - degree);

        bool within_range = false;
        int within_range_timer;

        while(time < timeout)
        {          
            double dist_traveled = odom.get_value() - last_odom_val;
            x += (dist_traveled) * sin(180 - imu.get_heading() * PI / 180);
            y += (dist_traveled) * cos(180 - imu.get_heading() * PI / 180);
            last_odom_val = odom.get_value();

            double error = target - odom.get_value();
            double speed = driveK.calculate(error);
            double correction = straightK.calculate(target_heading - imu.get_heading());
            
            chas.spin_left(speed + correction);
            chas.spin_right(speed - correction);

            if(time % 50 == 0) con.print(2, 0, "%.2f            ", error);

            if(abs(error) < 0.1)
            {
                break;
            }

            pros::delay(1);
            time += 1;
        }

        chas.stop();
        degree += imu.get_heading() - 180;
    }

    void rotate(double degrees, int timeout=3000, pros::Controller con=pros::Controller(pros::E_CONTROLLER_MASTER))
    {
        int time = 0;
        double target = degrees + degree;

        double start_heading = (degrees < 0 ? 330 : 30);
        imu.set_heading(start_heading);
        double target_heading = start_heading + degrees;

        while(time < timeout)
        {
            double dist_traveled = odom.get_value() - last_odom_val;
            x += (dist_traveled) * sin(180 - imu.get_heading() * PI / 180);
            y += (dist_traveled) * cos(180 - imu.get_heading() * PI / 180);
            last_odom_val = odom.get_value();

            double error = target_heading - imu.get_heading();
            double speed = rotateK.calculate(error);

            chas.spin_left(speed);
            chas.spin_right(-speed);

            if(time % 50 == 0) con.print(2, 0, "%.2f            ", target - chas.pos());

            if(abs(error) < 0.05)
            {
                break;
            }

            pros::delay(1);
            time += 1;
        }

        chas.stop();
        ideal_degree += degrees;
        degree += imu.get_heading() - 180;
    }
    
    void reset()
    {
        AutonController::reset();
        odom.reset();
        last_odom_val = 0;
        chas.reset();
    }
};

#endif