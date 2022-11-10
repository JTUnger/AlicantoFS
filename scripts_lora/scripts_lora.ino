

#include <SPI.h>

//Radio Head Library: 
#include <RH_RF95.h>

// We need to provide the RFM95 module's chip select and interrupt pins to the 
// rf95 instance below.On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 rf95(12, 6);

int LED = 13; //Status LED on pin 13

int packetCounter = 0; //Counts the number of packets sent
long timeSinceLastPacket = 0; //Tracks the time stamp of last packet received
// The broadcast frequency is set to 921.2, but the SADM21 ProRf operates
// anywhere in the range of 902-928MHz in the Americas.
// Europe operates in the frequencies 863-870, center frequency at 
// 868MHz.This works but it is unknown how well the radio configures to this frequency:
//float frequency = 864.1;
float frequency = 921.2;
unsigned long myTime = 0; 

//Definicion del bufffer
uint8_t data[RH_RF95_MAX_MESSAGE_LEN];

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//int nro_dec = sizeof("Latitud:####; Longitud:####");
int nro_dec = 43;

byte buf[RH_RF95_MAX_MESSAGE_LEN];
byte len = sizeof(buf);
void setup()
{
  //pinMode(LED, OUTPUT);

  Serial.begin(9600);

  while(!Serial);
  Serial.println("SERIAL OK");
  //Se espera a que haya una conexion serial.

  
  if (rf95.init() == false){
    while (1);
    //Se espera a que esté disponible la radio.

  }
  else{
    delay(500);
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }


// The default transmitter power is 13dBm, using PA_BOOST.///////////////
// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then /////////////////
// you can set transmitter powers from 5 to 23 dBm://////////////////////

//Se determina la potencia.

  rf95.setFrequency(frequency);  
  rf95.setTxPower(15, false);
}



void loop(){
  //1. Preguntar a Serial por Input
  //2. Si hay, enviar por radio.
  //3. Preguntar a radio por input.
  //4. Si hay, enviar por Serial


  //if (Serial.available())
  String input_serial = Serial.readString();
  Serial.flush();

  const uint8_t* output_lora = reinterpret_cast<const uint8_t*>(input_serial.c_str());
  rf95.send(output_lora, 100);
  rf95.waitPacketSent();

  //SerialUSB.println(input_serial);
  //SerialUSB.println(*output_lora);
  Serial.flush(); ///////////////////////////


  if (rf95.recv(buf, &len)) {

    rf95.recv(buf, &len);
    String input_lora = (char*)buf;
    input_lora.trim();


    Serial.println(input_lora);
    Serial.flush();
  
  }
delay(100);
}
