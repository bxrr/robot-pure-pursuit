#ifndef __AUTON__
#define __AUTON__

#include "main.h"
#include "global.h"
#include "obj/auton_obj.h"
#include <vector>
#include <string>

using namespace glb;
using namespace pros;

void test()
{
    aut.drive(1000);
    aut.rotate_to(90);
    aut.rotate_to(0);
    aut.drive(-1000);
}

void right()
{
    aut.rotate_to(-45);
    aut.rotate_to(0);
}

void left()
{
    aut.rotate_to(180);
    aut.rotate_to(0);
}

std::vector<Auton> autons
{
    Auton("test", test),
    Auton("right", right),
    Auton("left", left),
};

#endif