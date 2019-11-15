#include "main.h"
#include "config.h"

int speed;

void sleep (int X)
{
  pros::delay(X);
}

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

void setRightSpeed(int speed)
{
  EtuOikea.move(speed);
  TakaOikea.move(speed);
}

void setLeftSpeed(int speed)
{
  EtuVasen.move(speed);
  TakaVasen.move(speed);
}

void setNorthWestSpeed(int speed)
{
  EtuVasen.move(speed);
  TakaOikea.move(speed);
}

void setNorthEastSpeed(int speed)
{
  EtuOikea.move(speed);
  TakaVasen.move(speed);
}


void nostinLiike (int YA, int speed = 100)
{
  switch (YA)
  {
   case 1: Nostin.move(speed); break;
   case 2: Nostin.move(-speed); break;
   case 3: Nostin.move(0); break;
  }
}

void keraajaLiike (int suunta, int speed = 30) {
  switch(suunta) {

    case 1:
     KerainOikea.move(speed);
     KerainVasen.move(speed);
    break;

    case 2:
     KerainOikea.move(-speed);
     KerainVasen.move(-speed);
    break;

    case 3:
     KerainOikea.move(0);
     KerainVasen.move(0);
    break;
  }
}

void RN (int suunta, int speed = 40) {
  switch (suunta) {

    case 1:
     RampinNostin.move(speed);
    break;

    case 2:
     RampinNostin.move(-speed);
    break;

    case 3:
     RampinNostin.move(0);
    break;
  }
}

// define your global instances of motors and other devices here
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

double getDistance(double degrees) {
  return M_PI* 3.25* (degrees / 360);
}

double getHeading(bool U = true) {
  if (U) updateEncoders();
  double vr = getDistance(DEr);
  double vl = getDistance(DEl);
  return (-180* (vl - vr)) / ((dl - dr)* M_PI);
}

void kk () {
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


void turn (bool slow, float degree, int speed) {
  double raja = 0.2;
  int minSpeed = 5;
  Task mittaa(kk);
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


void G () {
 vex::thread mittaa(kk);

 while (1) {


   sleep(5);
  }
}
