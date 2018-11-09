/* Proyecto_3
 * Arquitectura de Computadoras I
 * Esteban Gonzalez Gutierrez
 * Jean Paul Jimenez Arias
 * Samanta Solano Acuña
 * ***********************Modulo Transmisor*************************
 * Este modulo se encargara de obtener las señales de los sensores
 * por medio de ADC que tiene en sus entradas. y enviarlas por medio
 * del modulo de transmision RadioHead69_RawDemo_tx.
 */

#include <SPI.h>
#include <RH_RF69.h>
#define waitTime 10000
/************ Configuracion del Radio ***************/

// Frecuencia a la que desea enviar el mensaje!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

// Instancia del driver del radio
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // Conteo de paquete, aumenta en cada transmision

/************ Variables necesarias ***************/
int photocellReading; 				// lectura del sensor luz analogica
int tempReading; 						  // lectura del sensor temperatura analogica
int humedadReading; 					// lectura del sensor de humedad analogica

int sensorLuz = A3;           // pin 6: entrada del sensor de luz 
int sensorTemp = A2;          // pin 9: entrada del sensor temperatura
int sensorHum = A1;           // pin 10: entrada del sensor humedad

String temp;                  //temperatura para transmitir
String luz;                   //luz para transmitir
String hum;								    //humedad para transmitir
String info;                  //Informacion para transmitir
char infoC[11];               //Informacion en Char


void setup() 
{
  Serial.begin(115200);           // Inicio de la consola serial
  //while (!Serial) { delay(1); } // Espera a que la consola este abierta

  pinMode(LED, OUTPUT);           // Selecciona el led de la placa como salida
  pinMode(RFM69_RST, OUTPUT);     // Selecciona el boton de Reset
  digitalWrite(RFM69_RST, LOW);   // Se establece el boton en LOW

  // Reseteo Manual
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("Inicio Fallido del radio RFM69");   //Mensaje de error por no iniciar el modulo
    while (1);
  }
  Serial.println("El radio RFM69 inicio correctamente");//Mensaje de inicializacion del modulo correcta
  // La frecuencia default es 434.0MHz, Modulacion GFSK_Rb250Fd250, +13dbM (Para modulo de baja potencia)
  // Sin encriptacion
  if (!rf69.setFrequency(RF69_FREQ)) {        //Si no se pudo setiar la frecuencia deseada
    Serial.println("Seteo de Frecuencia Fallido");    //Presenta un mensaje de error en la consola serial
  }

  // Si se esta usando un modulo RF69 de alta potencia Ej: RFM69HW, se debe setiar la potencia del Transmisor
  // con la ishighpowermodule flag setieada asi:
  rf69.setTxPower(20, true);  // rango desde 14-20 para potencia, 2do argumento debe ser cierto para 69HCW

  // Llave de encriptacion, debe ser la misma que en el receptor
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT); //Definicion del led de la placa como salida

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");  //Impresion de la frecuencia del Radio
}



void loop() {
  photocellReading = analogRead(sensorLuz);       //Lectura sensor de Iluminacion
  tempReading = analogRead(sensorTemp);           //Lectura sensor de Temperatura
  humedadReading = analogRead(sensorHum);         //Lectura sensor de Humedad
  delay(waitTime);                                // Transmite cada 10 segundos

	Serial.print("luz = ");					                //Impresion en monitor serial
	Serial.print(photocellReading); 		            // del sensor de iluminacion
	Serial.print(", temperatura = ");			          //Impresion en monitor serial
	Serial.print(tempReading); 				              // del sensor de temperatura
	Serial.print(", humedad = ");				            //Impresion en monitor serial
	Serial.println(humedadReading);		              // del sensor de humedad
  temp = String(tempReading, DEC);                //pasamos el dato a string, para enviarlo
  luz = String(photocellReading, DEC);            //pasamos el dato a string, para enviarlo
  hum = String(humedadReading, DEC);              //pasamos el dato a string, para enviarlo
  info="";                                        // Info = vacio
  info.concat(luz);                               // Info = iluminacion
  info.concat(",");
  info.concat(temp);                              //Info = iluminacion, temperatura
  info.concat(",");
  info.concat(hum);                               //Info = iluminacion, temperatura, humedad
  info.toCharArray(infoC,12);                     //Conversion de Info a arreglo Char
  Serial.println(infoC);                          //Impresion de Info en monitor serial
  
  // Envio del mensaje!
  rf69.send((uint8_t *)infoC, strlen(infoC));
  rf69.waitPacketSent();
  Blink(LED,50,3);
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {  //Funcion para que el led de la placa parpadee
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
