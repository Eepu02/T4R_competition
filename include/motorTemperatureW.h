#include "voidit.h"

int temperatureWarnings;
int line;

void displayVariables(int variable) {
  while(1)  {

     double takavasenTemp = LeftBackDrive.get_temperature();
     double kerainvasenTemp = CollectorLeft.get_temperature();
     double etuvasenTemp = LeftFrontDrive.get_temperature();
     double takaoikeaTemp = RightBackDrive.get_temperature();
     double kerainoikeaTemp = KerainOikea.get_temperature();
     double etuoikeaTemp = RightFrontDrive.get_temperature();
     double nostinTemp =Lift.get_temperature();
     double RNTemp =RampLift.get_temperature();


     if (takavasenTemp > 60)  {
        Controller1.set_text(1, 9,"LeftBackDrive");
        temperatureWarnings += 1;
      }
     else if (etuvasenTemp > 60){
        Controller1.set_text(1, 8,"LeftFrontDrive");
        temperatureWarnings += 1;
      }
     else if (takaoikeaTemp > 60) {
       Controller1.set_text(1, 9,"RightBackDrive");
       temperatureWarnings += 1;
      }
     else if (etuoikeaTemp > 60){
       Controller1.set_text(1, 8,"RightFrontDrive");
       temperatureWarnings += 1;
      }
     else if (RNTemp > 60) {
       Controller1.set_text(1, 12,"RampinNostin");
       temperatureWarnings += 1;
      }
     else if (nostinTemp > 60)  {
        Controller1.set_text(1, 6,"nostin");
        temperatureWarnings += 1;
     }
     else if (kerainvasenTemp > 60) {
        Controller1.set_text(1, 11,"CollectorLeft");
        temperatureWarnings += 1;
     }
     else if (kerainoikeaTemp > 60) {
        Controller1.set_text(1, 11,"KerainOikea");
        temperatureWarnings += 1;
     }
     else {
       Controller1.clear();
       temperatureWarnings = 0;
     }
     if (temperatureWarnings > 1) Controller1.set_text(2, 4, "lisaa");

     sleep(300);
  }
}
