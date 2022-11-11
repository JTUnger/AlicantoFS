// lora script for landing task
#include <SPI.h>
#include <RH_RF95.h>

// We need to provide the RFM95 module's chip select and interrupt pins to the
// rf95 instance below.On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 rf95(12, 6);

int LED = 13; //Status LED is on pin 13
int packetCounter = 0; //Counts the number of packets sent
long timeSinceLastPacket = 0; //Tracks the time stamp of last packet received
float frequency = 921.2; //Broadcast frequency

void setup()
{
  pinMode(LED, OUTPUT);

  SerialUSB.begin(9600);
  while(!SerialUSB);

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
  rf95.setTxPower(14, false);
}


void loop()
{
  // send data and status
  const char* cptr = SerialUSB.readStringUntil('\n').c_str();
  const byte* uptr = reinterpret_cast<byte*>(const_cast<char*>(cptr));
  packetCounter++;
  rf95.send(uptr, strlen(cptr));
  rf95.waitPacketSent();

  // wait for reply
  byte buf[RH_RF95_MAX_MESSAGE_LEN];
  byte len = sizeof(buf);

  if (rf95.waitAvailableTimeout(2000)) {
    if (rf95.recv(buf, &len)) {
      const char* outMsg = reinterpret_cast<char*>(const_cast<byte*>(buf));
      SerialUSB.println((char*)outMsg);
    }
  }
  delay(500);
}
