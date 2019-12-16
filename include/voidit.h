#include "main.h"
#include "config.h"

int speed;
int defaultTraySpeed = 100;
int defaultLiftSpeed = 100;
int defaultCollectorSpeed = 90;

// simple sleep function
void sleep (int x)
{
  pros::delay(x);
}

// resets all drive motor internal encoders
void resetDriveMotors ()
{
  EtuOikea.tare_position();
  TakaOikea.tare_position();
  EtuVasen.tare_position();
  TakaVasen.tare_position();
}

void setup() {
   KerainVasen.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   KerainOikea.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   Nostin.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   RampinNostin.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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

double avarage (float x, float y) {
  return (x + y) / 2;
 }


/*void displayDriveMotorSpeeds (int C)
{
 switch(C)
 {
   case 0:
     printf("vasenE %f\n", EtuVasen.getPosition());
   break;

   case 1:
     printf("vasenT %f\n", TakaVasen.getPosition());
   break;

   case 2:
    printf("OikeaE %fn", EtuOikea.getPosition());
   break;

   case 3:
    printf("OikeaT %f", TakaOikea.getPosition());
   break;

   default:
    printf(10, 20, "vasenE %f", EtuVasen.getPosition());
    printf(10, 60, "vasenT %f", TakaVasen.getPosition());
    printf(10, 100, "OikeaE %f", EtuOikea.getPosition());
    printf(10, 140, "OikeaT %f", TakaOikea.getPosition());
 }
}*/

// sets speed for right side drive
void setRightSpeed(int speed)
{
  EtuOikea.move(speed);
  TakaOikea.move(speed);
}

// sets speed for left side drive
void setLeftSpeed(int speed)
{
  EtuVasen.move(speed);
  TakaVasen.move(speed);
}

// sets speed for forward-left and backward-right motors
void setNorthWestSpeed(int speed)
{
  EtuVasen.move(speed);
  TakaOikea.move(speed);
}

// sets speeds for forward-right and backward-left motors
void setNorthEastSpeed(int speed)
{
  EtuOikea.move(speed);
  TakaVasen.move(speed);
}

void moveForward(int speed) {
  setRightSpeed(speed);
  setLeftSpeed(speed);
}

void movedBackward(int speed) {
  setRightSpeed(speed);
  setLeftSpeed(speed);
}

void moveRight(int speed) {
  setNorthWestSpeed(speed);
  setNorthEastSpeed(-speed);
}

void moveLeft(int speed) {
  setNorthWestSpeed(-speed);
  setNorthEastSpeed(speed);
}

void stop() {
  setRightSpeed(0);
  setLeftSpeed(0);
}

// function to control cube collector movement
void nostinLiike (int YA, int speed = defaultLiftSpeed)
{
  switch (YA)
  {
   case 1: Nostin.move(speed); break;
   case 2: Nostin.move(-speed); break;
   case 3: Nostin.move(0); break;
  }
}

// function to control cube collector movement
void keraajaLiike (int suunta, int speed = defaultCollectorSpeed) {
  switch(suunta) {
    case 1:   KerainOikea.move(speed);  KerainVasen.move(speed);  break;
    case 2:   KerainOikea.move(-speed); KerainVasen.move(-speed); break;
    case 3:   KerainOikea.move(0);      KerainVasen.move(0);      break;
    default:  printf("Error selecting case for keraajaLiike\n");  break;
  }
}

// function for controlling cube tray movement
void RN (int suunta, int speed = defaultTraySpeed) {
  switch (suunta) {
    case 1:   RampinNostin.move(speed);                 break;
    case 2:   RampinNostin.move(-speed);                break;
    case 3:   RampinNostin.move(0);                     break;
    default:  printf("Error selecting case for RN\n");  break;
  }
}

// Tracking wheels' distance to tracking center
// Values from CAD model
const float dr = 6.3202;
const float dl = 7.62301;
const float db = -3.125 + 1.25;

// Tracking variables
double er         = 0;
double el         = 0;
double eb         = 0;
double lastEr     = 0;
double lastEl     = 0;
double lastEb     = 0;
double DEr        = er - lastEr;
double DEl        = el - lastEl;
double DEb        = eb - lastEb;
double suunta     = 0.0;
double dSuunta    = 0.0;
double lastSuunta = 0.0;

double r = 0;

// Global x and y
double xG = 0;
double yG = 0;

// Local x and y
double xL = 0;
double yL = 0;

double error;

//float heading = 0.0;

void resetEncoders() {
  encoderBack.reset();
  encoderLeft.reset();
  encoderRight.reset();
}

// Updates current and previous encoder values
void getEncoderValues() {
  er = encoderRight.get_value();
  el = encoderLeft.get_value();
  eb = encoderBack.get_value();
  DEr = er - lastEr;
  DEl = el - lastEl;
  DEb = eb - lastEb;
  lastEr = er;
  lastEl = el;
  lastEb = eb;
}

double getDistance(double degrees, float d = 3.25) {
  return M_PI * d * (degrees / 360);
}

// Computes the change in orientation since last oriantation
double getHeading() {
  // Degress
  //return (-180 * (getDistance(DEl) - getDistance(DEr))) / ((dl - dr) * M_PI);

  //Radians
  return (getDistance(DEl) - getDistance(DEr)) / (dr + dl);
}

/* Local axis is offset from global axis by getHeading() / 2!!!*/
void track() {
 resetDriveMotors();
 resetEncoders();
 while (1) {
   /*------------------------------------------------------*/
   /*                                                      */
   /*                      NEW VALUES                      */
   /*                                                      */
   /*------------------------------------------------------*/

   // Gets latest encoder values
   getEncoderValues();

   // Compute change in orientation
   dSuunta = getHeading();

   // The new orientation is the previous orientation plus the change
   suunta += dSuunta;

   /*------------------------------------------------------*/
   /*                                                      */
   /*                  LOCAL COORDINATES                   */
   /*                                                      */
   /*------------------------------------------------------*/
/*
   // Check if the orientation is the same as last cycle
   if(lastSuunta == dSuunta) {
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
   /*if(suunta == 0) {
     xG += xL;
     yG += yL;
   }
   else {
     xG += xL * cos(dSuunta);
     yG += yL * sin(dSuunta);
   }*/

   lastSuunta = dSuunta;

   // Compute translation in local coordinates
   //xL = 2 * sin(getHeading() / 2) * (DEb / getHeading() + db);
   //yL = 2 * sin(getHeading() / 2) * (DEr / getHeading() + dr);

   // Compute translation in global coordinates
   //xG = xL * cos(getHeading() / 2) + yL * sin(getHeading() / 2);
   //yG = -xL * sin(getHeading() / 2) + yL * cos(getHeading() / 2);

   sleep(5);
 }
}
void Mittaus() {
    //pros::Task track();
    //std::thread mittaa (track);
}


void turn (bool slow, float degree, int speed) {
  double raja = 0.2;
  int minSpeed = 5;
  Mittaus();
  error = (suunta - degree);
  if (degree < error) {
    do {
     error = (suunta - degree);
     if (slow) {
       if(error < speed) speed = (fabs(error));
       if(speed < minSpeed) speed = minSpeed;
     }
     setRightSpeed(speed);
     setLeftSpeed(-speed);
    }
    while (error > raja);
    setLeftSpeed(0);
    setRightSpeed(0);
  }
  if (degree > error) {
    do {
      error = (suunta - degree);
      if (slow) {
       if(error < speed) speed = (fabs(error));
       if(speed < minSpeed) speed = minSpeed;
      }
      setRightSpeed(-speed);
      setLeftSpeed(speed);
    }
    while (error < raja);
    setLeftSpeed(0);
    setRightSpeed(0);
  }
}

//void my_task_fn(void* param) {
     //std::cout << Hello << (char*)param << std::endl;
     // ...
 //}

void G () {
  Mittaus();
 while (1) {
   sleep(5);
  }
}
