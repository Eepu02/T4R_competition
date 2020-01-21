#include "15scPGMS.h"
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

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

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

void autonomous()
{
	// Create tracking task with a priority of 10 (default 8);
	pros::Task track_task (track, (void*)"HELLO WORLD", 10,
								TASK_STACK_DEPTH_DEFAULT, "Tracking task");
	//autoUnfold();

	pros::Task my_cpp_task (track_task);

	// while(1) {
	// 	printSensorValues();
	// 	sleep(20);
	// }
	startIntake();
	forward(25, 0, 127);
	forward(10, 0, 60);
	forward(5, 0, 40);
	sleep(500);
	turn(-40);
	stopIntake();
	forward(-39, 0, 127);
	turn(-20);

	encoderBack.reset();
	moveRight(127);
	while(getDistance(eb) > -5) sleep(20);
	turn(-20);

	startIntake();
	forward(25, 0, 60);
	forward(-15, 0, 127);
	stopIntake();
	debug();
	// turn(80.5, 127,true); //80.5 oikealle, systemaattinen virhe
	// autoUnfold();
	//
	// startIntake();
	// moveForward(50);
	// sleep(1500);
	// stopIntake();
	// movedBackward(60);
	// sleep(1500);
	// turnLeft(60);
	// sleep(1600);
	// moveForward(60);
	// sleep(2000);
	// reverseIntake();
	// sleep(500);
	// turnLeft(60);
	// sleep(500);
	// movedBackward(100);
	// sleep(750);
	// stop();
	// stopIntake();


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

//	turn(90, 127);
	//sesamOpen();
	//read();
	while(1)	{
	 	// int potVal = PotRN.get_value();
	 	// printf("nostin: %d\n", Nostin.get_encoder_units());
 		tankD();
	}
}
