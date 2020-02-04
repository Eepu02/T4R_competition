#include "drivePGMS.h"


void blue1() {


}

void blue2() {

  	startIntake();
  	sleep(150);
  	KerainOikea.move(-127);
  	CollectorLeft.move(0);
  	sleep(250);
  	startIntake();
  	forward(20, 0, 127);
  	forward(15.3, 0, 100); //ensimmäiset kolme kutiota on kerätty

  	debug();
  	printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
  	turn(42.6);
  	movedBackward(127);
  	sleep(1000);
  	stop(); // liike neljän kuution eteen

  	forward(-37.2, 0, 127);
  	setLeftSpeed(-127);
  	sleep(290);
  	stop();
  	forward(12.4, 0, 79);
  	setRightSpeed(127);
  	setLeftSpeed(40);
  	sleep(150);
  	forward(22.5, 0, 45);
  	moveForward(45);
  	sleep(300);
  	stop(); //neljä kuutiota on kerätty

  	printf("Repositioning");
  	forward(-22.6, 0, 127);
  	resetEncoders();
  	debug();
  	printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
  	turn(-125);
  	debug();
  	printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
    // kääntynyt kohti stac aluetta

  	printf("Moving to stack position");

  	stack(); // stackkaus
    stop();
  	debug();

}

void red1() {

  startIntake();
	sleep(250);
	KerainOikea.move(-127);
	CollectorLeft.move(0);
	sleep(150);
	startIntake();

  forward(10, 0, 90);

  turn(32);

  forward(20, 32, 100);
  sleep(80);
  setRightSpeed(100);
  sleep(150);
  moveForward(120);
  sleep(250);
  stop();

  setRightSpeed(120);
  sleep(100);
  moveForward(120);
  sleep(100);
  stop();

  movedBackward(127);
  sleep(100);
  stop();
  // turn(-90);

  // forward(20, -90, 100);

}

void red2() {

  startIntake();
	forward(20, 0, 127);
	forward(15.3, 0, 100);

	debug();
	printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
	turn(-42.6);
	movedBackward(127);
	sleep(900);
	stop();
	forward(-37.2, 0, 127);
	setRightSpeed(-127);
	sleep(290);
	stop();


	forward(12.4, 0, 79);

	setLeftSpeed(127);
	setRightSpeed(40);
	sleep(150);
	forward(22.5, 0, 45);

	moveForward(45);
	sleep(300);
	stop();

	printf("Repositioning");
	forward(-22.6, 0, 127);
	resetEncoders();
	debug();
	printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
	turn(125);
	debug();
	printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));

	printf("Moving to stack position");

	stack();
  stop();
	debug();
}

void skillsPGM() {
  startIntake();
  sleep(150);
  KerainOikea.move(-127);
  CollectorLeft.move(0);
  sleep(250);
  startIntake();
  forward(20, 0, 127);
  forward(15.3, 0, 100); //ensimmäiset kolme kutiota on kerätty

  debug();
  printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
  turn(42.6);
  movedBackward(127);
  sleep(1000);
  stop(); // liike neljän kuution eteen

  forward(-37.2, 0, 127);
  setLeftSpeed(-127);
  sleep(290);
  stop();
  forward(12.4, 0, 79);
  setRightSpeed(127);
  setLeftSpeed(40);
  sleep(150);
  forward(22.5, 0, 45);
  moveForward(45);
  sleep(300);
  stop(); //neljä kuutiota on kerätty

  // printf("Repositioning");
  // forward(-22.6, 0, 127);
  // resetEncoders();
  // debug();
  // printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
  // turn(-125);
  // debug();
  // printf("Recalculated heading in degrees: %f\n", ((getDistance(el) - getDistance(er)) / (dr + dl)) * (180 / M_PI));
  // // kääntynyt kohti stac aluetta
  //
  // printf("Moving to stack position");
  //
  // stack(); // stackkaus
  // stop();
  debug();
}
