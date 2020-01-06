#include "voidit.h"

int temperatureWarnings;
int line;

void displayVariables(int variable) {
  while(1)  {

     double takavasenTemp = TakaVasen.get_temperature();
     double kerainvasenTemp = KerainVasen.get_temperature();
     double etuvasenTemp = EtuVasen.get_temperature();
     double takaoikeaTemp = TakaOikea.get_temperature();
     double kerainoikeaTemp = KerainOikea.get_temperature();
     double etuoikeaTemp = EtuOikea.get_temperature();
     double nostinTemp =Nostin.get_temperature();
     double RNTemp =RampinNostin.get_temperature();


     if (takavasenTemp > 60)  {
        C1.set_text(1, 9,"takavasen");
        temperatureWarnings += 1;
      }
     else if (etuvasenTemp > 60){
        C1.set_text(1, 8,"EtuVasen");
        temperatureWarnings += 1;
      }
     else if (takaoikeaTemp > 60) {
       C1.set_text(1, 9,"TakaOikea");
       temperatureWarnings += 1;
      }
     else if (etuoikeaTemp > 60){
       C1.set_text(1, 8,"EtuOikea");
       temperatureWarnings += 1;
      }
     else if (RNTemp > 60) {
       C1.set_text(1, 12,"RampinNostin");
       temperatureWarnings += 1;
      }
     else if (nostinTemp > 60)  {
        C1.set_text(1, 6,"nostin");
        temperatureWarnings += 1;
     }
     else if (kerainvasenTemp > 60) {
        C1.set_text(1, 11,"kerainvasen");
        temperatureWarnings += 1;
     }
     else if (kerainoikeaTemp > 60) {
        C1.set_text(1, 11,"KerainOikea");
        temperatureWarnings += 1;
     }
     else {
       C1.clear();
       temperatureWarnings = 0;
     }
     if (temperatureWarnings > 1) C1.set_text(2, 4, "lisaa");

     sleep(300);
  }
}
