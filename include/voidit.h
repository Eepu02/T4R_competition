#include "main.h"
#include "config.h"

int speed;
int defaultTraySpeed = 40;
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
  pros::delay(20);
}

double avarage (float x, float y) {
  return (x+y) /2;
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

const float dr = 6.3202;//6.34375; //+ 1.25;
const float dl = -7.62301;//-7.65625; //+ 1.25;
//const float db = -3.125 + 1.25;

double er = 0;
double el = 0;
double eb = 0;
double lastEr = er;
double lastEl = el;
double lastEb = eb;
double DEr = er - lastEr;
double DEl = el - lastEl;
double DEb = eb - lastEb;
double suunta = 0.0;

double r = 0;
double x = 0;
double y = 0;

double error;

//float heading = 0.0;

void resetEncoders() {
  encoderBack.reset();
  encoderLeft.reset();
  encoderRight.reset();
}

// Updates current and previous encoder values
void updateEncoders() {
  lastEr = er;
  lastEl = el;
  lastEb = eb;
  er = encoderRight.get_value();
  el = encoderLeft.get_value();
  eb = encoderBack.get_value();
  DEr = er - lastEr;
  DEl = el - lastEl;
  DEb = eb - lastEb;
}

double getDistance(double degrees, float r = 3.25) {
  return M_PI* r* (degrees / 360);
}

double getHeading(bool U = true) {
  if (U) updateEncoders();
  double vr = getDistance(DEr);
  double vl = getDistance(DEl);
  return (-180* (vl - vr)) / ((dl - dr)* M_PI);
}

void kk (void* p) {
 resetDriveMotors();
 resetEncoders();
 while (1) {
   sleep(5);
   printf( "suunta, %f\n", suunta);
   printf( " error, %f\n", error);
   suunta = suunta + getHeading();
   r = (getDistance(DEl)* 180) / M_PI* getHeading(false);

   double gamma = 180 - (getHeading(false) / 2) - 90;
   double betta = getHeading(false) / 2;
   double z =  2* (r* sin(betta));

   x = z* cos(gamma) + x;
   y = z* sin(gamma) + y;

   printf( "x, %f\n", x);
   printf( "y, %f\n", y);
   printf( "r, %f\n", r);
   printf( "z, %f\n", z);
   printf( "gamma, %f\n", gamma);
   printf( "betta, %f\n", betta);

 }
}
void Mittaus() {
    std::string text("PROS");
    pros::Task mittaa(kk, &text, "p");
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
