#include "main.h"
#include "global.h"
#include "driver.h"

using namespace glb;
using namespace pros;


void initialize() 
{
	pros::lcd::initialize();
	
}

void autonomous() {}

void opcontrol() 
{
	int time = 0;
	
	while(true)
	{
		arcade_drive();
		print_info(time);

		if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			aut_controller.drive_to(1000, 1000, 90);

		delay(1);
		time++;
	}
}
