#include <SPI.h>
//#include <RH_RF69.h>

int photocellPin = 6;						// pin 6: entrada del sensor de luz 
int tempPin = 9; 							// pin 9: entrada del sensor temperatura
int humedadPin = 10; 						// pin 10: entrada del sensor humedad

int photocellReading; 						// lectura del sensor luz analogica
int tempReading; 							//lectura del sensor temperatura analogica
int humedadReading; 						// lectura del sensor de humedad analogica

char A;										//luz;
char B;										//temp;
char C;									    //humedad;
char info;
void setup(void) {
	Serial.begin(9600);
}
void loop(void) {
photocellReading = analogRead(photocellPin);
	Serial.print("luz = ");					// para vigilar los datos, no es necesario
	Serial.print(photocellReading); 		// the raw analog reading
	Serial.print("temperatura = ");			// para vigilar los datos, no es necesario
	Serial.print(tempReading); 				// the raw analog reading
	Serial.print("humedad = ");				// para vigilar los datos, no es necesario
	Serial.print(humedadReading);		    // the raw analog reading
A=char(photocellReading);
B=char(tempReading);
C=char(humedadReading);
info=A+B+C;
delay(10000);//10 segundos
}
