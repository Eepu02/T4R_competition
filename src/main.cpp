#include "eitoimi500.h"

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
	printf("hi");
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
<<<<<<< Updated upstream
void autonomous() {
win = false;
eneble = false;
}
=======
void autonomous() {}
>>>>>>> Stashed changes
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
void opcontrol() {

  int speed = 80;
	int Nspeed = 100;
  bool slowM = false;
  resetDriveMotors();
  float NK = 0.2;
	 track();

do RN(1); while(PotRN.get_value() > 1800);

  while(1) {
arcadeDrive();
   //vex::thread koordinaatit(G);
   //displayDriveMotorSpeeds(4);

	 // main driving code
  /* if (abs(Controller1.get_analog(ANALOG_RIGHT_Y)) > 5 ||
	 		 abs(Controller1.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(Controller1.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(Controller1.get_analog(ANALOG_LEFT_Y));
   }*/
/*
	 else if (Controller1.get_analog(ANALOG_LEFT_X) > 5 ){
     moveRight(abs(Controller1.get_analog(ANALOG_LEFT_X)));
	 }
	 else if (Controller1.get_analog(ANALOG_LEFT_X) < 5 ){
     moveLeft(abs(Controller1.get_analog(ANALOG_LEFT_X)));
	 }
	 */
	/* else if(Controller1.get_digital(DIGITAL_RIGHT)) moveRight(speed);
   else if(Controller1.get_digital(DIGITAL_LEFT)) moveLeft(speed);
   else stop();
*/
    //kauhan varsien liike
		//movement of collector liftx

    if (Controller1.get_digital(DIGITAL_L1)) nostinLiike(1);
    else if (Controller1.get_digital(DIGITAL_L2) && !Bumper.get_value()) nostinLiike(2);

		else if (Controller1.get_digital(DIGITAL_UP) && // kun RN liikkuu ylös nostin nousee myös
		 					PotRN.get_value() > 1500 && Bumper.get_value()) {
			if(PotRN.get_value() < 1800) nostinLiike(1);
		}
		else if (Controller1.get_digital(DIGITAL_DOWN) && !Bumper.get_value()) {  //kun RN liikkuu alas nostin laskee
				nostinLiike(2);
		}
    else nostinLiike(3);



    // kerääjän liike
		// collector movement
    if (Controller1.get_digital(DIGITAL_R1)) keraajaLiike(1);
    else if (Controller1.get_digital(DIGITAL_R2)) keraajaLiike(2);
    else keraajaLiike(3);


    // rampin nostimen liike
		// cube tray movement
		if (PotRN.get_value() > 2800 &&  Controller1.get_digital(DIGITAL_UP)) Nspeed = Nspeed - NK;
		else Nspeed = 127;
    if (Controller1.get_digital(DIGITAL_UP) && PotRN.get_value() < 4095 ) RN(1, Nspeed);
    else if (Controller1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > 1800) RN(2, Nspeed);
    else RN(3);

		//printSensorValues();

   	sleep(20);

		//toinen ohjain, jos käytämme
 }
}
