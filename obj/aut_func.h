#ifndef __AUT_FUNC__
#define __AUT_FUNC__

#include "main.h"
#include "chassis.h"
#include "pid.h"


class AutonFunc
{
private:
    Chassis &chas;
    PID driveK;
    PID rotateK;
    double degree;

public:
    AutonFunc(Chassis chas, PID init_driveK=PID(1, 0, 0), PID init_rotateK=PID(1, 0, 0))
    {
        double drive(double distance, double max_speed=120, int timeout=3500, double multiplier=1.0, int within_range_exit=100)
        {
            mtr::Mode mode = glb::PTO.status() ? mtr::all : mtr::chas;
            mtr::set_brake(mtr::coast, mode);

            glb::imu.set_heading(180);
            double ideal_heading = 180 + (ideal_degree - global_heading);
            float slew = 0.05;
            
            double target = mtr::pos() + distance;
            double error = target - mtr::pos();
            double integral = 0;
            double last_error;

            double left_start = mtr::left_pos();
            double right_start = mtr::right_pos();

            bool within_range = false;
            double within_range_err = 1.5;
            int within_range_time;

            bool negative = distance < 0;
            bool start_integral = false;

            int time = 0;
            // control loop
            while(time < timeout)
            {
                // update variables
                last_error = error;
                error = target - mtr::pos();
                integral += (start_integral) ? error : 0;
                double derivative = error - last_error;
                
                double straight_error = ideal_heading - glb::imu.get_heading();
                
                if(negative && error > 0)
                {
                    start_integral = true;
                }
                else if(!negative && error < 0)
                {
                    start_integral = true;
                }

                // speed variables
                double base_speed = error * DRIVE_KP + integral * DRIVE_KI + derivative * DRIVE_KD;
                slew = (slew < 1) ? (slew + 0.05) : (1);
                if(abs(base_speed) > max_speed) base_speed = multiplier * slew * (base_speed > 0 ? max_speed : -max_speed);

                double correction_speed = abs(base_speed / 120) * (straight_error * IMU_DRIVE_KP);
                correction_speed = (correction_speed > 50) ? 50 : correction_speed;
                
                // apply speeds
                mtr::spin_left(base_speed + correction_speed, mode);
                mtr::spin_right(base_speed - correction_speed, mode);

                // print error
                if(time % 50 == 0) { glb::con.print(0, 0, "Err: %lf        ", ideal_degree); }

                // check for exit
                if(abs(error) <= within_range_err || (abs(error) < 300 && abs(glb::left_back.get_actual_velocity()) < 2))
                {
                    if(!within_range)
                    {
                        within_range = true;
                        within_range_time = time;
                    }
                    else if(within_range_time + within_range_exit <= time)
                    {
                        break;
                    }
                }
                else within_range = false;

                // increment time
                pros::delay(5);
                time += 5;
            }
            // stop motors once out of loop
            mtr::stop(mode);
            global_heading += glb::imu.get_heading() - 180;
            return error;
        }
};

#endif