#include "main.h"
#include "voidit.h"


void liikuPisteeseen(int x, int y, int nopeus);

int koordinaattiX = 0;
int koordinaattiY = 0;
int robotinAsento = 0;

void liikuPisteeseen(int x, int y, int nopeus) {
	int asento = robotinAsento;
	int SivuttainNopeus = 0;
	int reunaMoottorienNopeus = 0;
	//x-akseli
	resetEncoders();
	int muutosX = x - koordinaattiX;  //lasketaan paljonko pitaa liikkua x-suuntaan
	bool liikkuukoX = muutosX != 0;  //jos muutos on eru suuri kun 0, liikutaan x-akselin suuntaisesti
	if(muutosX > 0) {  //jos muutos on positiivinen, liikutaan vasemmalle
		switch(asento) {  //x-akselin suuntaisesti liikuttaessa moottorit ja niiden suunnat riippuvat asennosta
      case 0:  SivuttainNopeus = nopeus;         break; //normaali asento
			case 1:  reunaMoottorienNopeus = nopeus;   break; //90 asteetta vastapaivaan kaantyneena
			case 2:  SivuttainNopeus = -nopeus;        break; //180 astetta kaantyneena
			case 3:  reunaMoottorienNopeus = -nopeus;  break; //90 asteetta myotapaivaan kaantyneena
		}
	}
	else if(muutosX < 0) {  //jos muutos on positiivinen, liikutaan oikealle
		switch(asento) {
			case 0: SivuttainNopeus = -nopeus;       break;
			case 1: reunaMoottorienNopeus = -nopeus; break;
			case 2: SivuttainNopeus = nopeus;        break;
			case 3: reunaMoottorienNopeus = nopeus;  break;
		}
	}
	//y-akseli
	int muutosY = koordinaattiY - y;  //lasketaan paljonko pitaa liikkua y-suuntaan
	bool liikkuukoY = muutosY != 0;  //jos muutos on eru suuri kun 0, liikutaan x-akselin suuntaisesti
	if(muutosY > 0) {  //jos muutos on positiivinen, liikutaan eteenpain
		switch(asento) {  //y-akselin suuntaisesti liikuttaessa moottorit ja niiden suunnat riippuvat asennosta
			case 0: reunaMoottorienNopeus = nopeus;  break; //normaali asento
			case 1: SivuttainNopeus = -nopeus;       break; //90 asteetta vastapaivaan kaantyneena
			case 2: reunaMoottorienNopeus = -nopeus; break; //180 astetta kaantyneena
			case 3: SivuttainNopeus = nopeus;        break; //90 asteetta myotapaivaan kaantyneena
		}
	}
	if(muutosY < 0) {  //jos muutos on positiivinen, liikutaan taaksepain
		switch(asento) {  //y-akselin suuntaisesti liikuttaessa moottorit ja niiden suunnat riippuvat asennosta
			case 0:  reunaMoottorienNopeus = -nopeus;    break; //normaali asento
			case 1:  SivuttainNopeus = nopeus;           break; //90 asteetta vastapaivaan kaantyneena
			case 2:  reunaMoottorienNopeus = nopeus;     break; //180 astetta kaantyneena
			case 3:  SivuttainNopeus = -nopeus;          break; //90 asteetta myotapaivaan kaantyneena
		}
	}
	int xNyt = 0;
	int yNyt = 0;
	//odotetaan etta molemmat moottorin eivat liiku
	while(liikkuukoX | liikkuukoY) {


		int gyroPoikkeama = suunta - (robotinAsento * 90);
    if (SivuttainNopeus > 0) moveRight(abs(SivuttainNopeus));
    else moveLeft(abs(SivuttainNopeus));
		//setMotor(keskiRengas, SivuttainNopeus);
    setLeftSpeed(reunaMoottorienNopeus - gyroPoikkeama);
    setRightSpeed(reunaMoottorienNopeus + gyroPoikkeama);
		//setMotor(vasenRengas, reunaMoottorienNopeus - gyroPoikkeama);
		//setMotor(oikeaRengas, reunaMoottorienNopeus + gyroPoikkeama);
		switch(asento) {	//laskureiden lukemat riippuvat asennosta
			case 0:
				xNyt = encoderBack.get_value(); // encoderBack.get_value();  //luetaan moottoreiden laskurit
				yNyt =  avarage(el, er);
				break;
			case 1:
				xNyt = avarage(el, er);
				yNyt = -encoderBack.get_value();
				break;
			case 2:
				xNyt = -encoderBack.get_value();
				yNyt = -avarage(el, er);
				break;
			case 3:
				xNyt = -avarage(el, er);
				yNyt = encoderBack.get_value();
				break;
		}
		//jos muutos on positiivinen ja enkooderi on enemman kuin muutos, tai jos muutos on negatiivinen ja enkooderi on vaheman kuin muutos, pysahdytaan
		if(((muutosX > 0) & (xNyt > muutosX)) | ((muutosX < 0) & (xNyt < muutosX))) {
			switch(asento) {
				case 0:
				case 2:
					stop();
					break;
				case 1:
				case 3:
					stop();
					break;
			}
			liikkuukoX = false;
		}
		//sama homma y-akselilla
		if(((muutosY > 0) & (yNyt > muutosY)) | ((muutosY < 0) & (yNyt < muutosY))) {
			switch(asento) {
				case 0:
				case 2:
				 stop();
					break;
				case 1:
				case 3:
					stop();
					break;
			}
			liikkuukoY = false;
		}
	}
stop();
	//tallennetaan uudet koordinaatit paikan muuttujiin
	//if(anturi == 0) {
		koordinaattiX = x;
		koordinaattiY = y;
	//}
	/*else {
		koordinaattiX += xNyt;
		koordinaattiY += yNyt;
	}*/
}
