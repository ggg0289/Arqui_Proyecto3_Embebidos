#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

//--------------Defina los valores de corte de voltaje, en voltios------------//
 #define ilumRef    1.5
 #define humeRef    1.5
 #define tempRef    1.5

// Frecuencia a la que desea enviar el mensaje!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

 const int yellowLed = 9;   // pin xx: Salida led amarillo (Iluminacion)
 const int greenLed = 10;   // pin xx: Salida led verde (temperatura)
 const int redLed = 6;      // pin xx: Salida led rojo (humedad)
 int l;                     // Variable Iluminacion
 int t;                     // Variable Temperatura
 int h;                     // Varible Humedad
 
// Instancia del driver del radio
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // Conteo de paquete, aumenta en cada transmision


void setup() 
{
  Serial.begin(115200);           // Inicio de la consola serial
  //while (!Serial) { delay(1); } // Espera a que la consola este abierta
  pinMode(yellowLed,OUTPUT);      // Definicion de salida led amarillo
  pinMode(greenLed,OUTPUT);       // Definicion de salida led verde
  pinMode(redLed,OUTPUT);         // Definicion de salida led rojo
  pinMode(LED, OUTPUT);           // Definicion de salida led de la placa
  pinMode(RFM69_RST, OUTPUT);     // Definicion de salida boton de reset
  
  digitalWrite(RFM69_RST, LOW);   // Se establece boton de reset en LOW

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
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("Configuracion de Frecuencia Fallido");
  }

  // Si se esta usando un modulo RF69 de alta potencia Ej: RFM69HW, se debe setiar la potencia del Transmisor
  // con la ishighpowermodule flag setieada asi:
  rf69.setTxPower(20, true);  // rango desde 14-20 para potencia, 2do argumento debe ser cierto para 69HCW

  // Llave de encriptacion, debe ser la misma que en el receptor  
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);       //Definicion del led de la placa como salida

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");  //Impresion de la frecuencia del Radio
}

 float fc = 0.0032226;      //Factor de conversion 
 
 void loop() {
 if (rf69.available()) {
    // Recibiendo el mensaje enviado por el modulo de transmision   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      Serial.print("Se recibió [");
      Serial.print(len);
      Serial.print("]caracteres: ");
      Serial.println((char*)buf);
      Serial.print("RSSI(fuerza de la señal): ");
      Serial.println(rf69.lastRssi(), DEC);
      
      String recibido = (char*)buf;               // Se asigna a la variable recibido el mensaje
      char recibidoC[12];                         // Variable Char para el mensaje
      recibido.toCharArray(recibidoC,12);         // Conversion a arreglo Char 
      /* 
       * En el siguiente condigo se separa la entrada que contiene los datos del mensaje
       * en sus componentes respectivas, por lo que a la variable i se le asigna el valor
       * del mensaje correspondiente al sensor de Iluminacion, a la variable t se le 
       * asugna el valor correspondiente del sensor de Temperatura y a la variable h se 
       * le asigna el valor correpondiente del senso de Humedad.
       */
      if(!isAlphaNumeric(recibidoC[1])){
        if(!isAlphaNumeric(recibidoC[3])){
          if(!isAlphaNumeric(recibidoC[5])){
            char temp[3]={recibidoC[4]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[6])){
            char temp[3]={recibidoC[4],recibido[5]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[4],recibido[5],recibido[6]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[2]};
          t = atoi(temp);
          }
        else if(!isAlphaNumeric(recibidoC[4])){
          if(!isAlphaNumeric(recibidoC[6])){
            char temp[3]={recibidoC[5]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[7])){
            char temp[3]={recibidoC[5],recibido[6]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[5],recibido[6],recibido[7]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[2],recibido[3]};
          t = atoi(temp);
          }
        else{
          if(!isAlphaNumeric(recibidoC[7])){
            char temp[3]={recibidoC[6]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[8])){
            char temp[3]={recibidoC[6],recibido[7]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[6],recibido[7],recibido[8]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[2],recibido[3],recibido[4]};
          t = atoi(temp);
          }  
       
        char luz[3]={recibidoC[0]};
        l = atoi(luz);
        }
      else if(!isAlphaNumeric(recibidoC[2])){
        if(!isAlphaNumeric(recibidoC[4])){
          if(!isAlphaNumeric(recibidoC[6])){
            char temp[3]={recibidoC[5]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[7])){
            char temp[3]={recibidoC[5],recibido[6]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[5],recibido[6],recibido[7]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[3]};
          t = atoi(temp);
          }
        else if(!isAlphaNumeric(recibidoC[5])){
          if(!isAlphaNumeric(recibidoC[7])){
            char temp[3]={recibidoC[6]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[8])){
            char temp[3]={recibidoC[6],recibido[7]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[6],recibido[7],recibido[8]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[3],recibido[4]};
          t = atoi(temp);
          }
        else{
          if(!isAlphaNumeric(recibidoC[8])){
            char temp[3]={recibidoC[7]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[9])){
            char temp[3]={recibidoC[7],recibido[8]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[7],recibido[8],recibido[9]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[3],recibido[4],recibido[5]};
          t = atoi(temp);
          } 
        char luz[3]={recibidoC[0],recibido[1]};
        l = atoi(luz);
        
      }else{
        if(!isAlphaNumeric(recibidoC[5])){
          if(!isAlphaNumeric(recibidoC[7])){
            char temp[3]={recibidoC[6]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[8])){
            char temp[3]={recibidoC[6],recibido[7]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[6],recibido[7],recibido[8]};
            h = atoi(temp);
            }
          char temp[3]={recibido[4]};
          t = atoi(temp);
          }
        else if(!isAlphaNumeric(recibidoC[6])){
          if(!isAlphaNumeric(recibidoC[8])){
            char temp[3]={recibidoC[7]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[9])){
            char temp[3]={recibidoC[7],recibido[8]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[7],recibido[8],recibido[9]};
            h = atoi(temp);
            }
          char temp[3]={recibido[4],recibido[5]};
          t = atoi(temp);
          }
        else{
          if(!isAlphaNumeric(recibidoC[9])){
            char temp[3]={recibidoC[8]};
            h = atoi(temp);
            }
          else if(!isAlphaNumeric(recibidoC[10])){
            char temp[3]={recibidoC[8],recibido[9]};
            h = atoi(temp);
            }
          else{
            char temp[3]={recibidoC[8],recibido[9],recibido[10]};
            h = atoi(temp);
            }
          char temp[3]={recibidoC[4],recibido[5],recibido[6]};
          t = atoi(temp);
          }  
        char luz[3]={recibidoC[0],recibidoC[1], recibidoC[2]};
        l = atoi(luz);
        }
      /*
       * Se llaman a las funciones correspondientes para imprimir los datos en consola
       * y encender los Leds respectivos
       */
      imprimir(l, t, h);    //Imprime los datos en Consola
      comparador(l, t, h);  //Apaga o enciendo los Leds
      Blink(LED, 40, 3);    //Llama a la funcion Blink
      
    } else {
      Serial.println("Recepcion Fallida");  //Mensaje de error por recepcion fallida
    }
  }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {   //Funcion para que el led de la placa parpadee
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
void imprimir(int i, int t, int h){
  Serial.print("Sensor de iluminacion: ");    // Imprime en consola la tension del sensor de iluminacion en voltios
  Serial.print(i*fc);
  Serial.println(" [V]");
  Serial.print("Sensor de temperatura: ");    // Imprime en consola la tension del sensor de temperatura
  Serial.print(t*fc);
  Serial.println(" [V]");
  Serial.print("Sensor de temperatura: ");    // Imprime en consola la tension del sensor de humedad
  Serial.print(h*fc);
  Serial.println(" [V]");
  Serial.println("");
}
void comparador(int i, int t, int h){
  if( i*fc <= ilumRef){                   // Si la señal de iluminacion es menor que la ref
    digitalWrite(greenLed, HIGH);         // se enciende el led amarillo
  }
  else{
    digitalWrite(greenLed, LOW);
  }
  if(t*fc >= tempRef){                    // Si la señal de temperatura es mayor que la ref
    digitalWrite(yellowLed, HIGH);        // se enciende el led verde
  }
  else{
    digitalWrite(yellowLed, LOW);
  }
  if(h*fc >= humeRef){                    // Si la señal de humedad es menor que la red
    digitalWrite(redLed, HIGH);           // se enciende el led rojo
  }
  else{
    digitalWrite(redLed, LOW);
  }
}
