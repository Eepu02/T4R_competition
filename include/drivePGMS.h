#include "voidit.h"


//variables for controller1
bool firConLift = true;
bool firConRN = true;
bool firConCollector = true;
bool firConDriv = true;
bool firConDrivSide = true;


// variables fo second controller
bool secConLift = true;
bool secConRN = true;
bool secConCollector = true;
bool SecConDriv = false;
bool SecConDrivSide = false;


// rampin nostimen muuttujat
int rnSpeed;
int maxVal = 3410; //max valur of RN
int hidVal = 2360;
int alVal = 1050;

int CoS()  {
  return round((maxVal - PotRN.get_value()) * 0.091);
}


//nostimen muuttujat
int Nspeed = 100;


//tank ajon muuttujat
int speed = 127;


//arcade ajon muuttujat
int Y1, X1;         //Vertical, Horizontal Joystick Values for Controller1
int Y2, X2;         // Vertical, Horizontal Joystick Values for Controller2
int rotation1;      //rotation1 Joystick Values for Controller1
int rotation2;     // rotation1 Joystick Values for Controller2
double deadband = 20;   // Threshold value for deadzone


void tankD()  {

  resetDriveMotors();


   // Tank driving code
   if (firConDriv && abs(Controller1.get_analog(ANALOG_RIGHT_Y)) > 5 |
       abs(Controller1.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(Controller1.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(Controller1.get_analog(ANALOG_LEFT_Y));
   }
   else if (SecConDriv && abs(Controller2.get_analog(ANALOG_RIGHT_Y)) > 5 |
            abs(Controller2.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(Controller2.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(Controller2.get_analog(ANALOG_LEFT_Y));
   }
   else if (firConDrivSide && Controller1.get_digital(DIGITAL_RIGHT)) moveRight(speed);
   else if (SecConDrivSide && Controller2.get_digital(DIGITAL_RIGHT)) moveRight(speed);

   else if (firConDrivSide && Controller1.get_digital(DIGITAL_LEFT)) moveLeft(speed);
   else if (SecConDrivSide && Controller2.get_digital(DIGITAL_LEFT)) moveLeft(speed);

   else stop();


    //kauhan varsien liike
    //movement of collector liftx

    if (firConLift && Controller1.get_digital(DIGITAL_L1) && Lift.get_encoder_units() < 3000) raiseLift();
    else if (secConLift && Controller2.get_digital(DIGITAL_L1)) raiseLift();

    else if (firConLift && Controller1.get_digital(DIGITAL_L2) && !Bumper.get_value()) lowerLift();
    else if (secConLift && Controller2.get_digital(DIGITAL_L2) &&
             !Bumper.get_value()) lowerLift();

    else stopLift();


    // kerääjän liike
    // collector movement

    if (PotRN.get_value() > hidVal) rnSpeed = CoS();
    else rnSpeed = 127;

    if (firConCollector && Controller1.get_digital(DIGITAL_R1)) startIntake(rnSpeed);
    else if (secConCollector &&  Controller2.get_digital(DIGITAL_R1)) startIntake(rnSpeed);



    else if (firConCollector && Controller1.get_digital(DIGITAL_R2)) reverseIntake();
    else if (secConCollector && Controller2.get_digital(DIGITAL_R2)) reverseIntake();

    else if (firConCollector && (PotRN.get_value() > 3200 && Controller1.get_digital(DIGITAL_R2))) reverseIntake(40);
    else if (secConCollector && (PotRN.get_value() > 3200 && Controller2.get_digital(DIGITAL_R2))) reverseIntake(40);

    // else if (firConCollector && Controller1.get_digital(DIGITAL_A)) keraajaLiike(2, 40);
    // else if (secConCollector && Controller2.get_digital(DIGITAL_A)) keraajaLiike(2, 40);

    else stopIntake();


    // rampin nostimen liike

    if(PotRN.get_value() > hidVal)  rnSpeed = CoS();
    else rnSpeed = 127;

    if(firConRN && Controller1.get_digital(DIGITAL_UP) &&  PotRN.get_value() < maxVal) raiseTray(rnSpeed);
    else if(secConRN && Controller2.get_digital(DIGITAL_UP) && PotRN.get_value() < maxVal) raiseTray(rnSpeed);

    else if(firConRN && Controller1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) lowerTray(127);
    else if(secConRN && Controller2.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) lowerTray(127);

    else stopTray();

    sleep(20);

}


void arcadeDrive() {


       // Get value of three joysticks used for speed and
       // direction.

      //arcade driving code

      if (firConDriv) {  Y1 = Controller1.get_analog(ANALOG_RIGHT_Y); // Vertical   axis
                         X1 = Controller1.get_analog(ANALOG_RIGHT_X);}  // Horizontal axis

      else if (SecConDriv) {  Y2 = Controller2.get_analog(ANALOG_RIGHT_Y); // Vertical   axis
                              X2 = Controller2.get_analog(ANALOG_RIGHT_X);}  // Horizontal axis<

      if (firConDriv) rotation1 = Controller1.get_analog(ANALOG_LEFT_X); // rotation1   axis
      else if (SecConDriv) rotation2 = Controller2.get_analog(ANALOG_LEFT_X);

      // Implement dead zones to compensate for joystick values
      // not always returning to zero
      if (firConDriv && abs(Y1) < deadband) Y1 = 0;
      else if (SecConDriv && abs(Y2) < deadband) Y2 = 0;
      if (firConDriv && abs(X1) < deadband) X1 = 0;
      else if (SecConDriv && abs(X2) < deadband) X2 = 0;
      if (firConDriv && abs(rotation1) < deadband) rotation1 = 0;
      else if (SecConDriv && abs(rotation2) < deadband) rotation2 = 0;

      // Convert joystick values to motor speeds
      if (firConDriv)RightFrontDrive.move(Y1 - X1 - rotation1);
      else if (SecConDriv) RightFrontDrive.move(Y2 - X2 - rotation2);

       if (firConDriv)RightBackDrive.move(Y1 + X1 - rotation1);
       else if (SecConDriv) RightBackDrive.move(Y2 + X2 - rotation2);

      if (firConDriv)  LeftFrontDrive.move(Y1 + X1 + rotation1);
      else if (SecConDriv) LeftFrontDrive.move(Y2 + X2 + rotation2);

      if (firConDriv)  LeftBackDrive.move(Y1 - X1 + rotation1);
      else if (SecConDriv) LeftBackDrive.move(Y2 - X2 + rotation2);


      //kauhan varsien liike
      //movement of collector liftx
      Lift.set_encoder_units(MOTOR_ENCODER_DEGREES);

      if (firConLift && Controller1.get_digital(DIGITAL_L1) && Lift.get_encoder_units() < 3000) raiseLift();
      else if (secConLift && Controller2.get_digital(DIGITAL_L1)) lowerLift();

      else if (firConLift && Controller1.get_digital(DIGITAL_L2) && !Bumper.get_value()) lowerLift();
      else if (secConLift && Controller2.get_digital(DIGITAL_L2) &&
               !Bumper.get_value()) lowerLift();

      else if (firConLift && Controller1.get_digital(DIGITAL_UP) && // kun RN liikkuu ylös nostin nousee myös
               PotRN.get_value() > 1840 && Bumper.get_value()) {
                  if(PotRN.get_value() < 1900) raiseLift();
      }
      else if (secConLift && PotRN.get_value() > 1840 && Bumper.get_value()) {
                  if(PotRN.get_value() < 1900) raiseLift();
      }
      else stopLift();


      // kerääjän liike
      // collector movement


      if (firConCollector && Controller1.get_digital(DIGITAL_R1)) startIntake();
      else if (secConCollector &&  Controller2.get_digital(DIGITAL_R1)) startIntake();

      else if (firConCollector && Controller1.get_digital(DIGITAL_R2)) reverseIntake();
      else if (secConCollector && Controller2.get_digital(DIGITAL_R2)) reverseIntake();

      else if (firConCollector && Controller1.get_digital(DIGITAL_A)) reverseIntake(10);
      else if (secConCollector && Controller1.get_digital(DIGITAL_A)) reverseIntake(10);

      else stopIntake();


      // rampin nostimen liike

      if(PotRN.get_value() > hidVal)  rnSpeed = CoS();
      else rnSpeed = 127;

      if(firConRN && Controller1.get_digital(DIGITAL_UP) &&  PotRN.get_value() < maxVal) raiseTray(rnSpeed);
      else if(secConRN && Controller2.get_digital(DIGITAL_UP) && PotRN.get_value() < maxVal) raiseTray(rnSpeed);

      else if(firConRN && Controller1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) lowerTray(127);
      else if(secConRN && Controller2.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) lowerTray(127);

      else stopTray();


  sleep(20);
}
