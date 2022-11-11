  // lora script for SaR task on Drone
// only needs to send status and data 
#include <SPI.h>

#include <RH_RF95.h>

RH_RF95 rf95(12, 6);

int LED = 13; //Status LED is on pin 13

int packetCounter = 0; //Counts the number of packets sent
long timeSinceLastPacket = 0; //Tracks the time stamp of last packet received

float frequency = 930.0; //Broadcast frequency

void setup()
{
  pinMode(LED, OUTPUT);

  SerialUSB.begin(9600);
  while(!SerialUSB);

  //Initialize the Radio.
  if (rf95.init() == false){
    SerialUSB.println("Radio Init Failed - Freezing");
    while (1);
  }
  else{
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
  rf95.setFrequency(frequency);
  rf95.setTxPower(14, true);
}


void loop()
{
  //Send a message to the other radio
  const char* buf = SerialUSB.readStringUntil('\n').c_str();
  const byte* msgOut = reinterpret_cast<byte*>(const_cast<char*>(buf));
  rf95.send(msgOut, strlen(buf));
  rf95.waitPacketSent();
  delay(200);
}
