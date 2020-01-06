#include "drivePGMS.h"
//#include "icd.h"
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	//pros::lcd::initialize();
	//pros::lcd::print(0, "%d\n", encoderBack.get_value());
	setup();
	eneble = true;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	eneble = true;
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	win = true;
	eneble = false;
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
win = false;
eneble = false;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

//  void my_task_fn(void* param) {
//   while(1) {
// 		printf("hi\n");
// 		sleep(100);
// 	}
//   // ...
// }

void opcontrol() {

	//resetEncoders();

	// Create tracking task with a priority of 10 (default 8);
	pros::Task track_task (track, (void*)"HELLO WORLD", 10,
								TASK_STACK_DEPTH_DEFAULT, "Tracking task");
	//autoUnfold();

	pros::Task my_cpp_task (track_task);
	turn(true, 90, 127);
	//read();
	while(1)	{
	 	// int potVal = PotRN.get_value();
	 	// printf("nostin: %d\n", Nostin.get_encoder_units());
 		tankD();
	}
}
