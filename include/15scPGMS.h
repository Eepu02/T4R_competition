#include "drivePGMS.h"

void seessamaukene() {

  do raiseTray(50); while (PotRN.get_value() < 2500);
  do lowerTray(50); while (PotRN.get_value() > 1100);

  raiseLift(50);
  sleep(2000);
  lowerLift(50);
}


void sin1() {


}

void sin2() {


}

void pun1() {


}

void pun2() {


}
