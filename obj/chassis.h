#ifndef __CHAS__
#define __CHAS__

#include "main.h"
#include "pid.h"
#include <vector>

class Chassis
{
private:
    std::vector<pros::Motor> left_motors;
    std::vector<pros::Motor> right_motors;

public:
    Chassis(int left_motor_ports[], int right_motor_ports[], auto gearset=pros::E_MOTOR_GEARSET_06, bool reverse_spin=false
            PID init_driveK=PID(1, 0, 0), PID init_rotateK=PID(1, 0, 0))
    {
        for(int port : left_motor_ports)
            left_motors.push_back(pros::Motor(port, gearset, reverse_spin + port < 0));
        for(int port : right_motor_ports)
            right_motors.push_back(pros::Motor(port, gearset, reverse_spin + port > 0));
    }

    void spin_left(double speed)
    {
        for(pros::Motor motor : left_motors)
            motor = speed;
    }

    void spin_right(double speed)
    {
        for(pros::Motor motor : right_motors)
            motor = speed;
    }

    void spin(double speed)
    {
        spin_left(speed);
        spin_right(speed);
    }

    void stop()
    {
        spin(0);
    }

    void reset()
    {
        for(pros::Motor motor : left_motors)
            motor.tare_position();
        for(pros::Motor motor : right_motors)
            motor.tare_position();
        for(pros::ADIEncoder odom : odom_encoders)
    }

    double left_pos()
    {
        double sum = 0;
        for(pros::Motor motor : left_motors)
        {
            sum += motor.get_position();
        }
        return sum / left_motors.size();
    }

    double right_pos()
    {
        double sum = 0;
        for(pros::Motor motor : right_motors)
        {
            sum += motor.get_position();
        }
        return sum / left_motors.size();
    }
    
    double mtr_pos()
    {
        return (left_pos() + right_pos()) / 2
    }

    double temp()
    {
        double sum = 0;
        for(pros::Motor motor : left_motors)
            sum += motor.get_temperature();
        for(pros::Motor motor : right_motors)
            sum += motor.get_temperature();

        return sum / (left_motors.size() + right_motors.size());
    }
};

#endif