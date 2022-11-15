// lora script for SaR task from Ground Station
// only need to receive drone status and data   
#include <SPI.h>

#include <RH_RF95.h>

// We need to provide the RFM95 module's chip select and interrupt pins to the
// rf95 instance below.On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 rf95(12, 6);

int LED = 13; //Status LED on pin 13

int packetCounter = 0; //Counts the number of packets sent
long timeSinceLastPacket = 0; //Tracks the time stamp of last packet received
float frequency = 930.0;

void setup()
{
  pinMode(LED, OUTPUT);

  SerialUSB.begin(9600);
  while(!SerialUSB);
  SerialUSB.println("GS Server!");

  //Initialize the Radio.
  if (rf95.init() == false){
    SerialUSB.println("Radio Init Failed - Freezing");
    while (1);
  }
  else{
  // An LED indicator to let us know radio initialization has completed.
    SerialUSB.println("GS receiver up!");
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }

  rf95.setFrequency(frequency);
  //rf95.setTxPower(14, true);
}

void loop()
{
  if (rf95.available()){
    // Should be a message for us now
    byte buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)){
      digitalWrite(LED, HIGH); //Turn on status LED
      timeSinceLastPacket = millis(); //Timestamp this packet
      const char* msgIn;
      msgIn = reinterpret_cast<char*>(const_cast<byte*>(buf));
      SerialUSB.print((char*)msgIn);
      SerialUSB.println();
    }
  }
  //Turn off status LED if we haven't received a packet after 1s
  if(millis() - timeSinceLastPacket > 1000){
    digitalWrite(LED, LOW); //Turn off status LED
    timeSinceLastPacket = millis(); //Don't write LED but every 1s
  }
}
