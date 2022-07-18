#include "main.h"
#include "global.h"
#include "driver.h"
#include "auton.h"

using namespace glb;
using namespace pros;


Auton auton = autons.at(0);

void initialize() 
{
	lcd::initialize();
	auton = auton_selector(autons);
}

void autonomous()
{
	auton.run();
}

void opcontrol() 
{
	aut.reset();
	int time = 0;
	
	while(true)
	{
		arcade_drive();
		print_info(time, auton.get_name());

		if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			autonomous();
		if(con.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
			aut.reset();

		delay(1);
		time++;
	}
}
