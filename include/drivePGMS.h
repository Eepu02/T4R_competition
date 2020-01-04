#include "voidit.h"


bool secConLift = true;
bool secConRN = true;
bool secConCollector = true;
bool SecConDriv = true;


void tankD()  {

  int speed = 80;
  int Nspeed = 100;
  bool slowM = false;
  resetDriveMotors();
  float NK = 0.2;
   track();

do RN(1); while(PotRN.get_value() > 1800);

  while(1) {

   // main driving code
   if (abs(C1.get_analog(ANALOG_RIGHT_Y)) > 5 ||
       abs(C1.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(C1.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(C1.get_analog(ANALOG_LEFT_Y));
   }
   else if (SecConDriv && abs(C2.get_analog(ANALOG_RIGHT_Y)) > 5 |
       abs(C2.get_analog(ANALOG_LEFT_Y)) > 5) {
     setRightSpeed(C2.get_analog(ANALOG_RIGHT_Y));
     setLeftSpeed(C2.get_analog(ANALOG_LEFT_Y));
   }

   else if (C1.get_digital(DIGITAL_RIGHT)) moveRight(speed);
   else if (SecConDriv & C2.get_digital(DIGITAL_RIGHT)) moveRight(speed);

   else if (C1.get_digital(DIGITAL_LEFT)) moveLeft(speed);
   else if (SecConDriv & C2.get_digital(DIGITAL_LEFT)) moveLeft(speed);

   else stop();

    //kauhan varsien liike
    //movement of collector liftx

    if (C1.get_digital(DIGITAL_L1)) nostinLiike(1);
    else if (secConLift & C2.get_digital(DIGITAL_L1)) nostinLiike(1);

    else if (C1.get_digital(DIGITAL_L2) & !Bumper.get_value()) nostinLiike(2);
    else if (secConLift & C2.get_digital(DIGITAL_L2) &
              !Bumper.get_value()) nostinLiike(2);

    else if (C1.get_digital(DIGITAL_UP) && // kun RN liikkuu ylös nostin nousee myös
              PotRN.get_value() > 1500 && Bumper.get_value()) {
                if(PotRN.get_value() < 1800) nostinLiike(1);
              }
    else if (secConLift && PotRN.get_value() > 1500 && Bumper.get_value()) {
                if(PotRN.get_value() < 1800) nostinLiike(1);
              }

    else if (C1.get_digital(DIGITAL_DOWN) && !Bumper.get_value()) nostinLiike(2);  //kun RN liikkuu alas nostin laskee
    else if (secConLift &&  C2.get_digital(DIGITAL_DOWN) &&
              !Bumper.get_value()) nostinLiike(2);

    else nostinLiike(3);


    // kerääjän liike
    // collector movement
    if (C1.get_digital(DIGITAL_R1)) keraajaLiike(1);
    else if (secConCollector &&  C2.get_digital(DIGITAL_R1)) keraajaLiike(1);

    else if (C1.get_digital(DIGITAL_R2)) keraajaLiike(2);
    else if (secConCollector && C2.get_digital(DIGITAL_R2)) keraajaLiike(2);

    else keraajaLiike(3);


    // rampin nostimen liike
    // cube tray movement
    if (PotRN.get_value() > 2800 && C1.get_digital(DIGITAL_UP)) Nspeed = Nspeed - NK;
    else if (secConRN && PotRN.get_value() > 2800 &&
              C2.get_digital(DIGITAL_UP)) Nspeed = Nspeed-NK;
    else Nspeed = 127;

    if (C1.get_digital(DIGITAL_UP) && PotRN.get_value() < 4095 ) RN(1, Nspeed);
    else if (secConRN && PotRN.get_value() < 4095 &&
              C2.get_digital(DIGITAL_UP)) RN(1, Nspeed);

    else if (C1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > 1800) RN(2, Nspeed);
    else if (secConRN &&  PotRN.get_value() > 1800 &&
              C2.get_digital(DIGITAL_DOWN)) RN(2, Nspeed);
    else RN(3);


    sleep(20);
 }
}

void arcadeDrive() {

  int speed = 80;
  int Nspeed = 100;
  bool slowM = false;
  resetDriveMotors();
  float NK = 0.2;
   track();
    int Y1, X1;
    int Y2, X2;         // Vertical, Horizontal Joystick Values
    int rotation1;
    int rotation2;     // rotation1 Joystick Values
    int deadband = 20;   // Threshold value for deadzone

    do RN(1); while(PotRN.get_value() > 1800);

    while(1)  {

       // Get value of three joysticks used for speed and
                       // direction.
                       // Other platforms may have different code for this.

      Y1 = C1.get_analog(ANALOG_RIGHT_Y); // Vertical   axis
      X1 = C1.get_analog(ANALOG_RIGHT_X);  // Horizontal axis<

    if (SecConDriv)  Y2 = C2.get_analog(ANALOG_RIGHT_Y); // Vertical   axis
    if (SecConDriv)  X2 = C2.get_analog(ANALOG_RIGHT_X);  // Horizontal axis<

      rotation1 = C1.get_analog(ANALOG_LEFT_X); // rotation1   axis
      rotation2 = C2.get_analog(ANALOG_LEFT_X);

      // Implement dead zones to compensate for joystick values
      // not always returning to zero
      if (abs(Y1) < deadband) Y1 = 0;
      else if (SecConDriv && abs(Y2) < deadband) Y2 = 0;
      if (abs(X1) < deadband) X1 = 0;
      else if (SecConDriv && abs(X2) < deadband) X2 = 0;
      if (abs(rotation1) < deadband) rotation1 = 0;
      else if (SecConDriv && abs(rotation2) < deadband) rotation2 = 0;

      // Convert joystick values to motor speeds
      EtuOikea.move(Y1 - X1 - rotation1);
      if (SecConDriv) EtuOikea.move(Y2 - X2 - rotation2);

      TakaOikea.move(Y1 + X1 - rotation1);
       if (SecConDriv) TakaOikea.move(Y2 + X2 - rotation2);

      EtuVasen.move(Y1 + X1 + rotation1);
      if (SecConDriv) EtuVasen.move(Y2 + X2 + rotation2);

      TakaVasen.move(Y1 - X1 + rotation1);
      if (SecConDriv) TakaVasen.move(Y2 - X2 + rotation2);

      //kauhan varsien liike
      //movement of collector liftx

      if (C1.get_digital(DIGITAL_L1)) nostinLiike(1);
      else if (secConLift & C2.get_digital(DIGITAL_L1)) nostinLiike(1);

      else if (C1.get_digital(DIGITAL_L2) & !Bumper.get_value()) nostinLiike(2);
      else if (secConLift & C2.get_digital(DIGITAL_L2) &
                !Bumper.get_value()) nostinLiike(2);

      else if (C1.get_digital(DIGITAL_UP) && // kun RN liikkuu ylös nostin nousee myös
                PotRN.get_value() > 1500 && Bumper.get_value()) {
                  if(PotRN.get_value() < 1800) nostinLiike(1);
              }
      else if (secConLift && PotRN.get_value() > 1500 && Bumper.get_value()) {
                  if(PotRN.get_value() < 1800) nostinLiike(1);
                }

      else if (C1.get_digital(DIGITAL_DOWN) && !Bumper.get_value()) nostinLiike(2);  //kun RN liikkuu alas nostin laskee
      else if (secConLift &&  C2.get_digital(DIGITAL_DOWN) &&
                !Bumper.get_value()) nostinLiike(2);

      else nostinLiike(3);


      // kerääjän liike
      // collector movement
      if (C1.get_digital(DIGITAL_R1)) keraajaLiike(1);
      else if (secConCollector &&  C2.get_digital(DIGITAL_R1)) keraajaLiike(1);

      else if (C1.get_digital(DIGITAL_R2)) keraajaLiike(2);
      else if (secConCollector && C2.get_digital(DIGITAL_R2)) keraajaLiike(2);

      else keraajaLiike(3);


      // rampin nostimen liike
      // cube tray movement
      if (PotRN.get_value() > 2800 && C1.get_digital(DIGITAL_UP)) Nspeed = Nspeed - NK;
      else if (secConRN && PotRN.get_value() > 2800 &&
                C2.get_digital(DIGITAL_UP)) Nspeed = Nspeed-NK;
      else Nspeed = 127;

      if (C1.get_digital(DIGITAL_UP) && PotRN.get_value() < 4095 ) RN(1, Nspeed);
      else if (secConRN && PotRN.get_value() < 4095 &&
                C2.get_digital(DIGITAL_UP)) RN(1, Nspeed);

      else if (C1.get_digital(DIGITAL_DOWN) && PotRN.get_value() > 1800) RN(2, Nspeed);
      else if (secConRN &&  PotRN.get_value() > 1800 &&
                C2.get_digital(DIGITAL_DOWN)) RN(2, Nspeed);
      else RN(3);


      sleep(20);
    }
}
