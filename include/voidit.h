#include "config.h"
#include "main.h"



// Define default speeds for various robot mechanisms
int defaultTraySpeed = 127;
int defaultLiftSpeed = 127;
int defaultCollectorSpeed = 127;


pros::Mutex mutex;

// simple sleep function
void sleep (int x)
{
  pros::delay(x);
}

// Resets all tracking wheels
void resetEncoders() {
  encoderBack.reset();
  encoderLeft.reset();
  encoderRight.reset();
}

// Resets all drive motor internal encoders
void resetDriveMotors ()
{
  RightFrontDrive.tare_position();
  RightBackDrive.tare_position();
  LeftFrontDrive.tare_position();
  LeftBackDrive.tare_position();
  Lift.tare_position();
  Lift.set_encoder_units(MOTOR_ENCODER_DEGREES);
}

void setup() {
   CollectorLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   KerainOikea.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   Lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   RampLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   resetDriveMotors();
   resetEncoders();

}

// sensor debugging function
void printSensorValues() {
  printf("Encoder back: %d\n", encoderBack.get_value());
  printf("Encoder left: %d\n", encoderLeft.get_value());
  printf("Encoder right: %d\n", encoderRight.get_value());
  printf("Bumper: %d\n", Bumper.get_value());
  printf("Pot: %d\n", PotRN.get_value());
  pros::delay(100);
}

double average(float x, float y) {
  return (x + y) / 2;
 }

// sets speed for right side drive
void setRightSpeed(int speed)
{
  RightFrontDrive.move(speed);
  RightBackDrive.move(speed);
}

// sets speed for left side drive
void setLeftSpeed(int speed)
{
  LeftFrontDrive.move(speed);
  LeftBackDrive.move(speed);
}

// sets speed for forward-left and backward-right motors
void setNorthWestSpeed(int speed)
{
  LeftFrontDrive.move(speed);
  RightBackDrive.move(speed);
}

// sets speeds for forward-right and backward-left motors
void setNorthEastSpeed(int speed)
{
  RightFrontDrive.move(speed);
  LeftBackDrive.move(speed);
}

void moveForward(int speed) {
  setRightSpeed(speed);
  setLeftSpeed(speed);
}

void movedBackward(int speed) {
  setRightSpeed(-speed);
  setLeftSpeed(-speed);
}

void moveRight(int speed) {
  setNorthWestSpeed(speed);
  setNorthEastSpeed(-speed);
}

void moveLeft(int speed) {
  setNorthWestSpeed(-speed);
  setNorthEastSpeed(speed);
}

void turnRight(int speed) {
  setLeftSpeed(speed);
  setRightSpeed(-speed);
}

void turnLeft(int speed) {
  setLeftSpeed(-speed);
  setRightSpeed(speed);
}

void stop() {
  setRightSpeed(0);
  setLeftSpeed(0);
}

// function to control cube collector movement
void moveLift (int YA, int speed = defaultLiftSpeed)
{
  switch (YA)
  {
   case 1: Lift.move(speed);  break;
   case 2: Lift.move(-speed); break;
   case 3: Lift.move(0);      break;
  }
}

void raiseLift(int speed = defaultLiftSpeed) {
  moveLift(1, speed);
}

void lowerLift(int speed = defaultLiftSpeed) {
  moveLift(2, speed);
}

void stopLift() {
  moveLift(3);
}

// function to control cube collector movement
void moveIntake (int heading, int speed = defaultCollectorSpeed) {
  switch(heading) {
    case 1:   KerainOikea.move(speed);  CollectorLeft.move(speed);   break;
    case 2:   KerainOikea.move(-speed); CollectorLeft.move(-speed);  break;
    case 3:   KerainOikea.move(0);      CollectorLeft.move(0);       break;
    default:  printf("Error selecting case for moveIntake\n");     break;
  }
}

void startIntake(int speed = defaultCollectorSpeed) {
  moveIntake(1, speed);
}

void reverseIntake(int speed = defaultCollectorSpeed) {
  moveIntake(2, speed);
}

void stopIntake() {
  moveIntake(3);
}

// function for controlling cube tray movement
void moveTray (int heading, int speed = defaultTraySpeed) {
  switch (heading) {
    case 1:   RampLift.move(speed);                 break;
    case 2:   RampLift.move(-speed);                break;
    case 3:   RampLift.move(0);                     break;
    default:  printf("Error selecting case for RN\n");  break;
  }
}

void raiseTray(int speed = defaultTraySpeed) {
  moveTray(1, speed);
}

void lowerTray(int speed = defaultTraySpeed) {
  moveTray(2, speed);
}

void stopTray() {
  moveTray(3);
}

/* Function to unfold the robot automatically on match start */
void autoUnfold() {

  // setup
  Lift.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  RampLift.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  Lift.tare_position();
  RampLift.tare_position();

  // Fold out intakes
  do {
    raiseTray();
  } while(PotRN.get_value() < 3900); //1060
  reverseIntake(70);
  sleep(3000);
  stopIntake();

  RampLift.move_absolute(30, 127);

  /*while(!C1.get_digital(DIGITAL_X)) sleep(20);
  // Fold out tray
  RampLift.move_absolute(0, 127);
  raiseLift();
  while(Lift.get_position() < 1300) sleep(20);

  // Reset lift
  Lift.move_absolute(0, 127);
  sleep(5000);*/

}

// Tracking wheels' distance to tracking center
// Values from CAD model
const float dr = 6.3202;
const float dl = 7.62301;
const float db = -3.125;

// Tracking variables
int er         = 0;
int el         = 0;
int eb         = 0;
int lastEr     = 0;
int lastEl     = 0;
int lastEb     = 0;
int DEr        = er - lastEr;
int DEl        = el - lastEl;
int DEb        = eb - lastEb;
double heading      = 0.0;
double* ptr = &heading;
double deltaHeading = 0.0;
double lastSuunta   = 0.0;

double r = 0;

// Global x and y
double xG = 0;
double yG = 0;

// Local x and y
double xL = 0;
double yL = 0;

// double error;

//float heading = 0.0;

// Updates current and previous encoder values
void getEncoderValues() {
  er         = encoderRight.get_value();
  el         = encoderLeft.get_value();
  eb         = encoderBack.get_value();
  DEr        = er - lastEr;
  DEl        = el - lastEl;
  DEb        = eb - lastEb;
  lastEr     = er;
  lastEl     = el;
  lastEb     = eb;
}

double getDistance(int degrees, float d = 3.25) {
  return M_PI * d * (double(degrees) / 360);
}

// Computes the change in orientation since last oriantation
double getHeading() {
  // Degress
  //return (-180 * (getDistance(DEl) - getDistance(DEr))) / ((dl - dr) * M_PI);

  //Radians
  return (getDistance(DEl) - getDistance(DEr)) / (dr + dl);
}

/* Local axis is offset from global axis by getHeading() / 2!!!*/
void track(void* param) {
 double  globalX = 0;
 double  globalY = 0;
 double  radius  = 0;
 double lastHeading = 0;
 while (1) {
   /*------------------------------------------------------*/
   /*                                                      */
   /*                      NEW VALUES                      */
   /*                                                      */
   /*------------------------------------------------------*/

   // printf("kauha: %d\n", Lift.get_encoder_units());

   // Gets latest encoder values
   getEncoderValues();

   // Compute change in orientation
   deltaHeading = getHeading();

   if(fabs(deltaHeading) < 1) {

     // The new orientation is the previous orientation plus the change
     mutex.take(5);
     heading += deltaHeading;

     // printf("Kulma: %f\n", heading * (180 / M_PI));
     mutex.give();

     lastHeading = deltaHeading;
   }

   // double theta = (-180*())

   /*double gamma = M_PI - (M_PI/4) - (deltaHeading/2);

   if(deltaHeading == 0) radius = 0;
   else radius = (getDistance(DEr) / deltaHeading) + dr;

   double line = 2* (sin(deltaHeading/2)*radius);

   double currentX = line*cos(gamma);
   double currentY = line*sin(gamma);

   globalX += currentX;
   globalY += currentY;

   printf("Y: %f\n", globalY);
   printf("X: %f\n", globalX);

   //printf("X: %f\n", globalX);
   //printf("Y: %f\n", globalY);
   //printf(20, 160, "Y: %f", globalY);*/
   /*------------------------------------------------------*/
   /*                                                      */
   /*                  LOCAL COORDINATES                   */
   /*                                                      */
   /*------------------------------------------------------*/
/*
   // Check if the orientation is the same as last cycle
   if(lastSuunta == deltaHeading) {
     // Left and right wheel have moved the same distance, so current oriantation is 0
     yL = getDistance(DEr);

     // Local x-position is simply the back wheel's travel distance
     xL = getDistance(DEb);
   }
   else {
     // Compute chord lenght
     yL = 2 * sin(getHeading() / 2) * (DEr / getHeading() + dr);

     //Compute global x and y
     if(getHeading() == 0) {
       yG = getDistance(DEl);
     }
     else if(getHeading() < 0) {
       xG = yL * cos(M_PI - (getHeading() / 2) - (M_PI / 2));
     }
     else {
       xG = -yL * cos(M_PI - (getHeading() / 2) - (M_PI / 2));
     }
     yG = yL * sin(M_PI - (getHeading() / 2) - (M_PI / 2));
   }*/

   /*------------------------------------------------------*/
   /*                                                      */
   /*                  GLOBAL COORDINATES                  */
   /*                                                      */
   /*------------------------------------------------------*/

   // If global orientation is zero
   /*if(heading == 0) {
     xG += xL;
     yG += yL;
   }
   else {
     xG += xL * cos(deltaHeading);
     yG += yL * sin(deltaHeading);
   }*/

   // lastSuunta = deltaHeading;

   // Compute translation in local coordinates
   //xL = 2 * sin(getHeading() / 2) * (DEb / getHeading() + db);
   //yL = 2 * sin(getHeading() / 2) * (DEr / getHeading() + dr);

   // Compute translation in global coordinates
   //xG = xL * cos(getHeading() / 2) + yL * sin(getHeading() / 2);
   //yG = -xL * sin(getHeading() / 2) + yL * cos(getHeading() / 2);

   // Runs in 5 second intervals
   sleep(5);
 }
}

// double getDirection() {
//   // Gets latest encoder values
//   getEncoderValues();
//
//   // Compute change in orientation
//   deltaHeading = getHeading();
//
//   // The new orientation is the previous orientation plus the change
//   return heading += deltaHeading * (180 / M_PI);
// }

// void read() {
//   while(1) {
//     printf("Heading toisessa paikassa: %f\n", *ptr);
//     sleep(100);
//   }
// }

void turn (double targetHeading, int speed, bool slow = true) {
  double raja = 0.2;
  int minSpeed = 30;
  int nopeus;
  int constant;
  double error = targetHeading - heading * (180 / M_PI);

  do {
    error = targetHeading - heading * (180 / M_PI);
    printf("Error: %f ", error);
    printf("Heading: %f\n", heading * (180 / M_PI));

    if(error > 0) constant = 20;
    else if(error < 0) constant = -20;
    else constant = 0;
    nopeus = round(error * 1.3 + constant);
    if(nopeus < -127) nopeus = -127;
    else if(nopeus > 127) nopeus = 127;

    if(fabs(error) > 900) break;

    turnRight(nopeus);
    sleep(20);
  } while(fabs(error) > raja);

  printf("failed");
  stop();

  // if (targetHeading < error) {
  //   do {
  //   mutex.take(20);
  //    error = heading * (180 / M_PI) - targetHeading;
  //    mutex.give();
  //    if (slow) {
  //      if(fabs(error) < double(speed)) {
  //        speed = round(fabs(error)) * 6;
  //        if(speed < minSpeed) speed = minSpeed;
  //      }
  //    }
  //    turnLeft(speed);
  //    sleep(20);
  //   }
  //   while (fabs(error) > raja);
  //   stop();
  // }
  // if (targetHeading > error) {
  //   do {
  //     error = heading * (180 / M_PI) - targetHeading;
  //     // printf("Error: %f\n", error);
  //     if (slow) {
  //       if(fabs(error) < double(speed)) {
  //         speed = round(fabs(error));
  //         if(speed < minSpeed) speed = minSpeed;
  //       }
  //     }
  //     printf("speed: %d\n", speed);
  //     turnRight(speed);
  //     sleep(20);
  //   }
  //   while (fabs(error) > raja);
  //   stop();
  //   printf("Error: %f\n", error);
  // }
}

void PID(float target) {
  float kp = 0.0;
  float ki = 0.0;
  float kd = 0.0;

  float currentVal  = 0.0;
  float totalError  = 0.0;
  float speed       = 0.0;
  float lastError;

  while(1) {
    // Get latest values
    currentVal = average(getDistance(encoderLeft.get_value()), getDistance(encoderRight.get_value()));

    // Compute error for proportional term
    float error = target - currentVal;

    // Compute total error for integral term
    totalError += error;

    // Compute chane in error for derivative term
    float diffError = lastError - error;

    // Compute speed
    speed = error * kp + totalError * ki + diffError * kd;

    if(speed > 127) speed = 127;
    else if(speed < -127) speed = -127;

    moveForward(speed);

    sleep(20);

  }
}
double gyroValue;
//double error;
float kuljettumatka;
float currentDistance;
float uusiArvo;

void forward(float targetDistance, int angle, int speed, float speedScale = 0.97) {
    resetEncoders();
    getEncoderValues();
    float raja = 0.2;
    float lastDistance = 0;
    float currentDistance = average(getDistance(DEl), getDistance(DEr));

    float error = targetDistance - currentDistance;

    do {
      currentDistance = average(getDistance(DEl), getDistance(DEr));
      error = targetDistance - currentDistance;
      moveForward(error * 1.3);
      sleep(20);
    } while(fabs(error) > raja);
  //
  //   do {
  //     getEncoderValues();
  //
  //     currentDistance = average(getDistance(el), getDistance(er));
  //      gyroValue = heading;
  //      error = angle - gyroValue;
  //      if (currentDistance < targetDistance) sleep(10);
  //      else if (currentDistance > targetDistance) currentDistance = currentDistance - alkuarvo;
  //       moveForward(speed);
  //
  //     if (error > 0)
  //     {
  //         setLeftSpeed(speed);
  //         setRightSpeed(speed * speedScale);
  //     }
  //     else if (error < 0)
  //     {
  //         setLeftSpeed(speed * speedScale);
  //         setRightSpeed(speed);
  //      }
  //   else moveForward(speed);
  //
  //    double error = targetDistance - currentDistance;
  //
  //    if (error < 5) {
  //      if (error * 10 < speed) speed = error * 10;
  //    }
  //
  //    sleep(30);
  //
  //    printf("currentDistance: %f\n", currentDistance);
  //    printf("uusiArvo: %f\n", uusiArvo);
  //
  // }while (targetDistance >= currentDistance);
  // stop();
}
// drives straight backward using cm

void backward(float targetDistance, int angle, int speed, float speedScale = 0.97) {
  float error;
  resetEncoders();
  sleep(100);
  getEncoderValues();
  float alkuarvo = average(getDistance(el), getDistance(er));;
  printf("alkuarvo: %f\n", currentDistance);

  do {
    getEncoderValues();

    currentDistance = average(getDistance(el), getDistance(er));
     gyroValue = heading;
     error = angle - gyroValue;
     if (currentDistance < targetDistance) sleep(10);
     else if (currentDistance > targetDistance) currentDistance = currentDistance - alkuarvo;
      movedBackward(-speed);

    if (error > 0)
    {
        setLeftSpeed(-speed);
        setRightSpeed(-speed * speedScale);
    }
    else if (error < 0)
    {
        setLeftSpeed(-speed * speedScale);
        setRightSpeed(-speed);
     }
  else movedBackward(-speed);

   double error = targetDistance - currentDistance;

   if (error < 5) {
     if (error * -10 < speed) speed = error * -10;
   }

   sleep(30);

   printf("currentDistance: %f\n", currentDistance);
   printf("uusiArvo: %f\n", uusiArvo);

}while (targetDistance >= currentDistance);
stop();
}
