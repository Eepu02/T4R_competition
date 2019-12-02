#include "main.h"

pros::Controller Controller1(pros::E_CONTROLLER_MASTER);
pros::Motor TakaVasen(1);
pros::Motor KerainVasen(6, MOTOR_GEARSET_18);
pros::Motor EtuVasen(5);
pros::Motor TakaOikea(7, true);
pros::Motor KerainOikea(8, MOTOR_GEARSET_18, true);
pros::Motor EtuOikea(9, true);
pros::Motor Nostin(10, MOTOR_GEARSET_36, true);
pros::Motor RampinNostin(11);

pros::ADIEncoder encoderBack(1, 2);
pros::ADIEncoder encoderLeft(3, 4);
pros::ADIEncoder encoderRight(5, 6);
pros::ADIDigitalIn Bumper(7);
pros::ADIAnalogIn PotRN(8);

/*encoder encoderBack = encoder(Brain.ThreeWirePort.A);
encoder encoderLeft = encoder(Brain.ThreeWirePort.C);
encoder encoderRight = encoder(Brain.ThreeWirePort.E);
bumper Bumper = bumper(Brain.ThreeWirePort.G);
motor RampinNostin = motor(PORT12, ratio18_1, false);
pot PotRN = pot(Brain.ThreeWirePort.H);*/
