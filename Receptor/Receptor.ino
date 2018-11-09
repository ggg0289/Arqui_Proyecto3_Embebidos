#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

//--------------Defina los valores de corte de voltaje, en voltios------------//
 #define ilumRef    1.5
 #define humeRef    1.5
 #define tempRef    1.5
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif
 const int yellowLed = 9;  
 const int greenLed = 10; 
 const int redLed = 6;  
 int l;
 int t;
 int h;
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  pinMode(yellowLed,OUTPUT);
  pinMode(greenLed,OUTPUT);
  pinMode(redLed,OUTPUT);
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

 float fc = 0.0032226;
 
 void loop() {
 if (rf69.available()) {
    // Should be a message for us now   
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
      
      String recibido = (char*)buf;
      char recibidoC[12];
      recibido.toCharArray(recibidoC,12);
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
      imprimir(l, t, h);   //Imprime los datos en Consola
      comparador(l, t, h); //Apaga o enciendo los Leds
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
      
    } else {
      Serial.println("Receive failed");
    }
  }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
void imprimir(int i, int t, int h){
  Serial.print("Sensor de iluminacion: ");    //Imprime en consola la tension del sensor de iluminacion en voltios
  Serial.print(i*fc);
  Serial.println(" [V]");
  Serial.print("Sensor de temperatura: ");    ////Imprime en consola la tension del sensor de temperatura
  Serial.print(t*fc);
  Serial.println(" [V]");
  Serial.print("Sensor de temperatura: ");    ////Imprime en consola la tension del sensor de humedad
  Serial.print(h*fc);
  Serial.println(" [V]");
  Serial.println("");
}
void comparador(int i, int t, int h){
  if( i*fc >= ilumRef){           //Si la señal de iluminacion es mayor que la ref
    digitalWrite(greenLed, HIGH);//se enciende el led amarillo
  }
  else{
    digitalWrite(greenLed, LOW);
  }
  if(t*fc >= tempRef){           //Si la señal de temperatura es mayor que la ref
    digitalWrite(yellowLed, HIGH);         //se enciende el led verde
  }
  else{
    digitalWrite(yellowLed, LOW);
  }
  if(h*fc >= humeRef){               //Si la señal de humedad es menor que la red
    digitalWrite(redLed, HIGH);           //se enciende el led rojo
  }
  else{
    digitalWrite(redLed, LOW);
  }
}
