#ifndef __AUT_FUNC__
#define __AUT_FUNC__

#include "main.h"
#include "chassis.h"
#include "pid.h"


class AutonFunc
{
private:
    Chassis chas;
    pros::Imu imu;
    PID driveK;
    PID rotateK;
    PID straightK;
    double degree;
    double ideal_degree;

public:
    AutonFunc(Chassis init_chas, pros::Imu init_imu, PID init_driveK=PID(1, 0, 0), PID init_rotateK=PID(1, 0, 0), PID init_straightK=PID(0, 0, 0))
    : chas(init_chas), imu(init_imu), driveK(init_driveK), rotateK(init_rotateK), straightK(init_straightK) {
        degree = 0;
        ideal_degree = 0;
    }

    void drive(double distance, int timeout=5000, double max_speed=120, pros::Controller con=Controller(pros::E_CONTROLLER_MASTER))
    {
        int time = 0;
        double target = distance + chas.pos();

        imu.set_heading(180);
        double target_heading = 180 + (ideal_degree - degree);

        bool within_range = false;
        int within_range_timer;

        while(time < timeout)
        {
            double error = target - chas.pos();
            double speed = driveK.calculate(error, error / abs(error) == -distance / abs(distance));
            double correction = straightK.calculate(target_heading - imu.get_heading());
            
            chas.spin_left(speed + correction);
            chas.spin_right(speed - correction);

            if(time % 50 == 0) con.print(2, 0, "%.2f            ", error);

            if(abs(error) < 1.0)
            {
                if(!within_range)
                {
                    within_range_timer = time;
                    within_range = true;
                }
                else if(within_range_timer + 100 < time)
                    break;
            }
            else within_range = false;

            pros::delay(5);
            time += 5;
        }

        chas.stop();
        degree += imu.get_heading() - 180;
    }

    void rotate(double degrees, int timeout=3000, pros::Controller con=Controller(pros::E_CONTROLLER_MASTER))
    {
        int time = 0;
        double target = distance + chas.pos();

        double start_heading = (degrees < 0 ? 330 : 30);
        imu.set_heading(start_heading);
        double target_heading = start_heading + degrees;

        while(time < timeout)
        {
            double error = target_heading - imu.get_heading();
            double speed = rotateK.calculate(error);

            chas.spin_left(speed);
            chas.spin_right(-speed);

            if(time % 50 == 0) con.print(2, 0, "%.2f            ", target - chas.pos());

            if(abs(error) < 0.1)
            {
                if(!within_range)
                {
                    within_range_timer = time;
                    within_range = true;
                }
                else if(within_range_timer + 100 < time)
                    break;
            }
            else within_range = false;

            pros::delay(5);
            time += 5;
        }

        chas.stop();
        ideal_degree += degrees;
        degree += imu.get_heading() - 180;
    }
};

#endif