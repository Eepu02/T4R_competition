#include "config.h"
#include "main.h"



// Define default speeds for various robot mechanisms
int defaultTraySpeed = 127;
int defaultLiftSpeed = 127;
int defaultCollectorSpeed = 127;

// Minimum distance to be within target
float distanceTreshold = 0.2;
float speedScale = 0.97;
int maxVal = 3400; //max valur of RN

pros::Mutex mutex;

// simple sleep function
void sleep (int x)
{
  pros::delay(x);
}

// Resets all tracking wheels
void resetEncoders() {
  while(encoderBack.get_value() != 0 || encoderLeft.get_value() != 0 || encoderRight.get_value() != 0) {
    encoderBack.reset();
    encoderLeft.reset();
    encoderRight.reset();
    sleep(5);
  }
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
  pros::lcd::initialize();
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

int CoS(int pot)  {
  return round((maxVal - pot) * 0.091);
}

int aggCoS(int pot) {
  return round((maxVal - pot) * 0.15);
}

/* Function to unfold the robot automatically on match start */

// Tracking wheels' distance to tracking center
// Values from CAD model
const double dr = 6.3202;
const double dl = 7.62301;
const double db = 3.125;
const double dm = 6.98144;

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
double lastSuunta   = 0.0;


// Global x and y
double globalX = 0;
double globalY = 0;

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

double lastDist = 0;

double getDistance(int degrees, float d = 3.25) { //3.25, 3.5
  return M_PI * d * (double(degrees) / 360);
}
// Computes the change in orientation since last oriantation
double getHeading() {
  // Degress
  //return (-180 * (getDistance(DEl) - getDistance(DEr))) / ((dl - dr) * M_PI);

  //Radians
  double currentVal = (getDistance(el) - getDistance(er)) / (dr + dl);
  if(fabs(currentVal - lastSuunta) < 1) {
    lastSuunta = currentVal;
    return currentVal;
  }
  else return lastSuunta;
}

void advancedTrack(void* param) {

  double lastHeading = heading;
  double localX = 0;
  double localY = 0;
  double deltaX = 0;
  double deltaY = 0;

  while(1) {
    er         = encoderRight.get_value();
    el         = encoderLeft.get_value();
    eb         = encoderBack.get_value();
    DEr        = er - lastEr;
    DEl        = el - lastEl;
    DEb        = eb - lastEb;
    lastEr     = er;
    lastEl     = el;
    lastEb     = eb;

    double distLeft = getDistance(DEl);
    double distRight = getDistance(DEr);
    double distBack = getDistance(DEb);
    double totalDistLeft = getDistance(el);
    double totalDistRight = getDistance(er);
    double totalDistBack = getDistance(eb);

    // Compute absolute heading
    heading = (totalDistLeft - totalDistRight) / (dl + dr);
    double deltaHeading = heading - lastHeading;
    // printf("Delta heading: %f", deltaHeading);
    // printf("Heading: %f\n", heading);
    // If there is no change in heading
    if(deltaHeading == 0) {
      // printf("Heading is zero.\n");
      deltaX = distBack;
      deltaY = distRight;
    }

    // Otherwise compute new change in position
    else {
      deltaX = (2 * sin(deltaHeading / 2)) * ((distBack / deltaHeading) + db);
      deltaY = (2 * sin(deltaHeading / 2)) * ((distRight / deltaHeading) + dr);

      // printf("deltaX: %f	", deltaX);
      // printf("deltaY: %f\n", deltaY);

      double avgHeading = deltaHeading / 2;

      // printf("avgHeading: %f\n", avgHeading);

      //Convert to polar coordinates
      double polarRadius = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
      double polarTheta = atan(deltaY / deltaX);

      // Rotate by average rotation to counter assumption in previous caclulations
      polarTheta += avgHeading;

      //Convert back to cartesian coordinates
      deltaX = cos(polarTheta) * polarRadius;
      deltaY = sin(polarTheta) * polarRadius;
    }
    // printf("Deltay: %f\n", deltaY);

    globalX += deltaX;
    globalY += deltaY;

    double lastX;
    double lastY;

    // if(lastX != globalX) printf("Global X: %f	", globalX);
    // if(lastY != globalY) printf("Global Y: %f\n", globalY);

    lastX = globalX;
    lastY = globalY;

    lastHeading = heading;

    sleep(5);
  }
}

void printTrackingValues() {
  printf("Encoder right: %d\n", er);
  printf("Distance right: %f\n", getDistance(er));
  printf("Encoder left: %d\n", el);
  printf("Distance left: %f\n", getDistance(el));
  printf("Encoder back: %d\n", eb);
  printf("Distance back: %f\n", getDistance(eb));
  printf("Heading: %f\n", heading);
  printf("Heading in degrees: %f\n", heading * (180 / M_PI));
  printf("Global x: %f\n", globalX);
  printf("Global y: %f\n", globalY);
}

void debug() {
  printf("\n");
  printf("Sensor values:\n");
  printSensorValues();
  printf("\n");
  printf("Tracking values:\n");
  printTrackingValues();
}

void turn (double targetHeading, int speed = 127, bool slow = true) {
  double raja = 0.2;
  int minSpeed = 16;
  int nopeus;
  int constant;
  int increment = 0;
  double error = targetHeading - heading * (180 / M_PI);
  double lastError = error;

  do {
    error = targetHeading - heading * (180 / M_PI);

    if(error > 0) constant = 16;
    else if(error < 0) constant = -16;
    else constant = 0;
    nopeus = round(error * 1.5 + constant);
    if(nopeus < -127) nopeus = -127;
    else if(nopeus > 127) nopeus = 127;

    if(slow) {
      if(nopeus < minSpeed && nopeus > 0) nopeus = minSpeed;
      else if(nopeus > -minSpeed && nopeus < 0) nopeus = -minSpeed;
    }

    printf("Error: %f ", error);
    printf("Heading: %f ", heading * (180 / M_PI));
    printf("Speed: %d\n", nopeus);

    if(fabs(error) > 900) break;

    turnRight(nopeus);

    increment++;
    if(increment > 7 && lastError == error) {
      printf("Exiting for slow turn\n");
      break;
    }
    else if(increment > 7) {
      lastError = error;
      increment = 0;
    }
    sleep(20);
  } while(fabs(error) > raja);

  stop();
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

void forward(float targetDistance, int angle, int nopeus, bool deaccelerate = true) {
  resetEncoders();
  getEncoderValues();
  float lastDistance = 0;
  int minSpeed = 30;
  double speed;
  float currentDistance = 0;
  float error;
  int increment = 0;
  float lastError = error;

  do {
    currentDistance = average(getDistance(el), getDistance(er));
    error = targetDistance - currentDistance;
    speed = error * 10;
    if(speed > nopeus) speed = nopeus;
    else if(speed < -nopeus) speed = -nopeus;
    if(deaccelerate) {
      if(speed < minSpeed && speed > 0) speed = minSpeed;
      else if(speed > -minSpeed && speed < 0) speed = -minSpeed;
    }
    else minSpeed = nopeus;
    moveForward(speed);
    printf("Error: %f ", error);
    printf("Current distance: %f  ", currentDistance);
    printf("Speed: %f ", speed);
    printf("Encoder left: %d  ", el);
    printf("Encoder right: %d ", er);
    printf("Heading (deg): %f\n", heading * (180 / M_PI));


    increment++;
    if(increment > 7 && lastError == error) {
      printf("Exiting for slow movement\n");
      break;
    }
    else if(increment > 7) {
      lastError = error;
      increment = 0;
    }

    sleep(20);
  } while(fabs(error) > distanceTreshold);

  stop();
}

// A low level function to keep the robot straight while moving sideways
void lowLevelMoveSideways(float aste, int speed = 127) {

  // Compute speed adjustment to correct for invalid rotation
  double error = aste - heading * (180 / M_PI);
  int rotation = error * 10;

  // Convert speed values to motor speeds
  RightFrontDrive.move(speed - rotation);
  RightBackDrive.move(-speed - rotation);
  LeftFrontDrive.move(-speed + rotation);
  LeftBackDrive.move(speed + rotation);
}

// Moves the robot sideways while keeping it straight.
void moveSideways(float distance, float aste, int speed) {
  float error;
  do {
    error = distance - getDistance(eb);
    speed = error * 10;
    lowLevelMoveSideways(aste, speed);
    sleep(20);
  } while(fabs(error) > distanceTreshold);
}

void autoUnfold() {
  raiseTray();
  sleep(1300);
  reverseIntake();
  stopTray();
  sleep(1000);
  stopIntake();
  RampLift.move_absolute(150, 127);
}

void stack() {
  setLeftSpeed(110);
  setRightSpeed(110);
  reverseIntake(35);
  // stopIntake();
  // raiseTray(aggCoS(PotRN.get_value()));
  sleep(600);
  raiseTray(aggCoS(PotRN.get_value()));
  sleep(140);
  stop();
  sleep(450);
  stopIntake();

  // Raise tray
  do {
    raiseTray(aggCoS(PotRN.get_value()));
    sleep(20);
  } while(PotRN.get_value() < 3250); //1060
  stopTray();

  reverseIntake(127);
  sleep(150);
  moveForward(75);
  sleep(50);
  movedBackward(100);
  lowerTray();
  sleep(300);
  stopTray();
  stopIntake();
}
