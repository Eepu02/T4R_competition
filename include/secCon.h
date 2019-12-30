#include "main.h"
#include "config.h"
#include "voidit.h"

void secController () {

    int speed = 80;
    bool slowM = false;
    resetDriveMotors();
    //float NK = 0.8;

    while(1) {

     //vex::thread koordinaatit(G);
     //displayDriveMotorSpeeds(4);

  	 // main driving code
     if (abs(Controller1.get_analog(ANALOG_RIGHT_Y)) > 5 ||
  	 		 abs(Controller1.get_analog(ANALOG_LEFT_Y)) > 5) {
       setRightSpeed(Controller1.get_analog(ANALOG_RIGHT_Y));
       setLeftSpeed(Controller1.get_analog(ANALOG_LEFT_Y));
     }
     if(Controller1.get_digital(DIGITAL_RIGHT) || Controller2.get_digital(DIGITAL_RIGHT)) moveRight(speed);
     else if(Controller1.get_digital(DIGITAL_LEFT) || Controller2.get_digital(DIGITAL_LEFT)) moveLeft(speed);
     else stop();

      //kauhan varsien liike
  		//movement of collector lift
      if (Controller1.get_digital(DIGITAL_L1) ||
          Controller2.get_digital(DIGITAL_L1)) nostinLiike(1);
      else if ((Controller1.get_digital(DIGITAL_L2) ||
                Controller2.get_digital(DIGITAL_L2)) && !Bumper.get_value()) nostinLiike(2);
  		else if ((Controller1.get_digital(DIGITAL_UP) ||
                Controller2.get_digital(DIGITAL_UP)) && PotRN.get_value() > 1500) {
  			if(PotRN.get_value() < 1800) nostinLiike(1);
  		}
      else nostinLiike(3);

      // kerääjän liike
  		// collector movement
      if (Controller1.get_digital(DIGITAL_R1) ||
          Controller2.get_digital(DIGITAL_R1))  keraajaLiike(1);
      else if (Controller1.get_digital(DIGITAL_R2) ||
               Controller2.get_digital(DIGITAL_R2)) keraajaLiike(2);
      else keraajaLiike(3);


      // rampin nostimen liike
  		// cube tray movement
      if ((Controller1.get_digital(DIGITAL_UP) ||
           Controller2.get_digital(DIGITAL_UP))&& PotRN.get_value() < 4095 ) RN(1);
      else if ((Controller1.get_digital(DIGITAL_DOWN) ||
                Controller2.get_digital(DIGITAL_DOWN)) && PotRN.get_value() > 1550) RN(2);
      else RN(3);

      if (Controller2.get_digital(DIGITAL_Y))
  		//printSensorValues();

     	sleep(20);
    }
}
