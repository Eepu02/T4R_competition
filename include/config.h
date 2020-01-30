#include "main.h"

pros::Controller Controller1(pros::E_CONTROLLER_MASTER);
pros::Controller Controller2(pros::E_CONTROLLER_PARTNER);
pros::Motor LeftBackDrive(12);
pros::Motor CollectorLeft(15, MOTOR_GEARSET_18, true);
pros::Motor LeftFrontDrive(14);
pros::Motor RightBackDrive(20, true);
pros::Motor KerainOikea(7, MOTOR_GEARSET_18, false);
pros::Motor RightFrontDrive(10, true);
pros::Motor Lift(19, MOTOR_GEARSET_36, true);
pros::Motor RampLift(13, MOTOR_GEARSET_36);

pros::ADIEncoder encoderBack(1, 2);
pros::ADIEncoder encoderLeft(3, 4);
pros::ADIEncoder encoderRight(5, 6);
pros::ADIDigitalIn Bumper(7);
pros::ADIAnalogIn PotRN(8);
