#include "voidit.h"

//ensimmäisen ohjaimen muuttujat
bool firConLift = true;
bool firConRN = true;
bool firConCollector = true;
bool firConDriv = true;
bool firConDrivSide = true;


// toisen ohjaimen muuttujat
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
int Y1, X1;         //Vertical, Horizontal Joystick Values for C1
int Y2, X2;         // Vertical, Horizontal Joystick Values for C2
int rotation1;      //rotation1 Joystick Values for C1
int rotation2;     // rotation1 Joystick Values for C2
double deadband = 20;   // Threshold value for deadzone


void tankD()  {

  resetDriveMotors();


   // Tank driving code
   if (firConDriv && abs(C1.get_analog(ANALOG_RIGHT_Y)) > 5 |
       abs(C1.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(C1.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(C1.get_analog(ANALOG_LEFT_Y));
   }
   else if (SecConDriv && abs(C2.get_analog(ANALOG_RIGHT_Y)) > 5 |
            abs(C2.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(C2.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(C2.get_analog(ANALOG_LEFT_Y));
   }
   else if (firConDrivSide && C1.get_digital(DIGITAL_RIGHT)) moveRight(speed);
   else if (SecConDrivSide && C2.get_digital(DIGITAL_RIGHT)) moveRight(speed);

   else if (firConDrivSide && C1.get_digital(DIGITAL_LEFT)) moveLeft(speed);
   else if (SecConDrivSide && C2.get_digital(DIGITAL_LEFT)) moveLeft(speed);

   else stop();


    //kauhan varsien liike
    //movement of collector liftx
    Nostin.set_encoder_units(MOTOR_ENCODER_DEGREES);

    if (firConLift && C1.get_digital(DIGITAL_L1) && Nostin.get_encoder_units() < 3000) nostinLiike(1);
    else if (secConLift && C2.get_digital(DIGITAL_L1)) nostinLiike(1);

    else if (firConLift && C1.get_digital(DIGITAL_L2) && !Bumper.get_value()) nostinLiike(2);
    else if (secConLift && C2.get_digital(DIGITAL_L2) &&
             !Bumper.get_value()) nostinLiike(2);

    else if (firConLift && C1.get_digital(DIGITAL_UP) && // kun RN liikkuu ylös nostin nousee myös
             PotRN.get_value() > 1840 && Bumper.get_value()) {
                if(PotRN.get_value() < 1900) nostinLiike(1);
    }
    else if (secConLift && PotRN.get_value() > 1840 && Bumper.get_value()) {
                if(PotRN.get_value() < 1900) nostinLiike(1);
    }
    else nostinLiike(3);


    // kerääjän liike
    // collector movement


    if (firConCollector && C1.get_digital(DIGITAL_R1)) keraajaLiike(1);
    else if (secConCollector &&  C2.get_digital(DIGITAL_R1)) keraajaLiike(1);

    else if (firConCollector && C1.get_digital(DIGITAL_R2)) keraajaLiike(2);
    else if (secConCollector && C2.get_digital(DIGITAL_R2)) keraajaLiike(2);

    else if (firConCollector && (PotRN.get_value() > 3200 && C1.get_digital(DIGITAL_R2))) keraajaLiike(2, 40);
    else if (secConCollector && (PotRN.get_value() > 3200 && C2.get_digital(DIGITAL_R2))) keraajaLiike(2, 40);

    // else if (firConCollector && C1.get_digital(DIGITAL_A)) keraajaLiike(2, 40);
    // else if (secConCollector && C2.get_digital(DIGITAL_A)) keraajaLiike(2, 40);

    else keraajaLiike(3);


    // rampin nostimen liike

    if(PotRN.get_value() > hidVal)  rnSpeed = CoS();
    else rnSpeed = 127;

    if(firConRN && C1.get_digital(DIGITAL_UP) &&  PotRN.get_value() < maxVal) RN(1, rnSpeed);
    else if(secConRN && C2.get_digital(DIGITAL_UP) && PotRN.get_value() < maxVal) RN(1, rnSpeed);

    else if(firConRN && C1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) RN(2, 127);
    else if(secConRN && C2.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) RN(2, 127);

    else RN(3);

    sleep(20);

}


void arcadeDrive() {


       // Get value of three joysticks used for speed and
       // direction.

      //arcade driving code

      if (firConDriv) {  Y1 = C1.get_analog(ANALOG_RIGHT_Y); // Vertical   axis
                         X1 = C1.get_analog(ANALOG_RIGHT_X);}  // Horizontal axis

      else if (SecConDriv) {  Y2 = C2.get_analog(ANALOG_RIGHT_Y); // Vertical   axis
                              X2 = C2.get_analog(ANALOG_RIGHT_X);}  // Horizontal axis<

      if (firConDriv) rotation1 = C1.get_analog(ANALOG_LEFT_X); // rotation1   axis
      else if (SecConDriv) rotation2 = C2.get_analog(ANALOG_LEFT_X);

      // Implement dead zones to compensate for joystick values
      // not always returning to zero
      if (firConDriv && abs(Y1) < deadband) Y1 = 0;
      else if (SecConDriv && abs(Y2) < deadband) Y2 = 0;
      if (firConDriv && abs(X1) < deadband) X1 = 0;
      else if (SecConDriv && abs(X2) < deadband) X2 = 0;
      if (firConDriv && abs(rotation1) < deadband) rotation1 = 0;
      else if (SecConDriv && abs(rotation2) < deadband) rotation2 = 0;

      // Convert joystick values to motor speeds
      if (firConDriv)EtuOikea.move(Y1 - X1 - rotation1);
      else if (SecConDriv) EtuOikea.move(Y2 - X2 - rotation2);

       if (firConDriv)TakaOikea.move(Y1 + X1 - rotation1);
       else if (SecConDriv) TakaOikea.move(Y2 + X2 - rotation2);

      if (firConDriv)  EtuVasen.move(Y1 + X1 + rotation1);
      else if (SecConDriv) EtuVasen.move(Y2 + X2 + rotation2);

      if (firConDriv)  TakaVasen.move(Y1 - X1 + rotation1);
      else if (SecConDriv) TakaVasen.move(Y2 - X2 + rotation2);


      //kauhan varsien liike
      //movement of collector liftx
      Nostin.set_encoder_units(MOTOR_ENCODER_DEGREES);

      if (firConLift && C1.get_digital(DIGITAL_L1) && Nostin.get_encoder_units() < 3000) nostinLiike(1);
      else if (secConLift && C2.get_digital(DIGITAL_L1)) nostinLiike(1);

      else if (firConLift && C1.get_digital(DIGITAL_L2) && !Bumper.get_value()) nostinLiike(2);
      else if (secConLift && C2.get_digital(DIGITAL_L2) &&
               !Bumper.get_value()) nostinLiike(2);

      else if (firConLift && C1.get_digital(DIGITAL_UP) && // kun RN liikkuu ylös nostin nousee myös
               PotRN.get_value() > 1840 && Bumper.get_value()) {
                  if(PotRN.get_value() < 1900) nostinLiike(1);
      }
      else if (secConLift && PotRN.get_value() > 1840 && Bumper.get_value()) {
                  if(PotRN.get_value() < 1900) nostinLiike(1);
      }
      else nostinLiike(3);


      // kerääjän liike
      // collector movement


      if (firConCollector && C1.get_digital(DIGITAL_R1)) keraajaLiike(1);
      else if (secConCollector &&  C2.get_digital(DIGITAL_R1)) keraajaLiike(1);

      else if (firConCollector && C1.get_digital(DIGITAL_R2)) keraajaLiike(2);
      else if (secConCollector && C2.get_digital(DIGITAL_R2)) keraajaLiike(2);

      else if (firConCollector && C1.get_digital(DIGITAL_A)) keraajaLiike(2, 10);
      else if (secConCollector && C1.get_digital(DIGITAL_A)) keraajaLiike(2, 10);

      else keraajaLiike(3);


      // rampin nostimen liike

      if(PotRN.get_value() > hidVal)  rnSpeed = CoS();
      else rnSpeed = 127;

      if(firConRN && C1.get_digital(DIGITAL_UP) &&  PotRN.get_value() < maxVal) RN(1, rnSpeed);
      else if(secConRN && C2.get_digital(DIGITAL_UP) && PotRN.get_value() < maxVal) RN(1, rnSpeed);

      else if(firConRN && C1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) RN(2, 127);
      else if(secConRN && C2.get_digital(DIGITAL_DOWN) && PotRN.get_value() > alVal) RN(2, 127);

      else RN(3);


  sleep(20);
}
