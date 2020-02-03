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
	resetEncoders();

	pros::Task track_task (track, (void*)"HELLO WORLD", 10,
								TASK_STACK_DEPTH_DEFAULT, "Tracking task");

	pros::Task my_cpp_task (track_task);

	// pros::Task skills_task (skillsPGM, (void*)"HELLO WORLD", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Skills auto");

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
	//autoUnfold();
	// red2();
	red2();
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
// pros::Task my_cpp_task2 (skills_task);
// while(!Controller1.get_digital(DIGITAL_X)) sleep(20);
// my_cpp_task2().remove();
// skillsPGM();
while(1) {
	tankD();
}
}
