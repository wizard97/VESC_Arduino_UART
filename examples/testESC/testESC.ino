#include <PacketSerial.h>
#include "data_structures.h"
#include <HC05.h>
#include <VescSerial.h>
#include "SoftwareSerial.h"

#define ESC_BAUD_RATE 9600

void escMsgHandler(VescSerial &vesc, COMM_PACKET_ID type, void *msg);
static uint32_t start = millis();
//Debugging
SoftwareSerial s_esc1(10, 11);
// end debug

VescSerial esc1(s_esc1, &escMsgHandler);

unsigned char tmp[50]; //tmp buffer

void setup() {
 
  esc1.begin(ESC_BAUD_RATE); //start esc's
 
  Serial.begin(38400);
  Serial.println("Starting loop");
  //esc1.requestValues();
}

void loop() {

  if (millis()-start >= 100) {  
    Serial.println("request");
    esc1.requestValues();
    start = millis();
  }
  
  /*
  for (uint32_t i = 1000; i < 40000; i += 100 )
    esc1.setRPM(i);

    for (uint32_t i = 40000; i >= 1000; i -= 100 )
    esc1.setRPM(i);
    */
  esc1.service();
  //esc1.requestValues();
  //esc1.service();
  //esc1.requestVersion();
  
}




void escMsgHandler(VescSerial &vesc, COMM_PACKET_ID type, void *msg)
{
  //esc1.requestValues();
  if (type == COMM_GET_VALUES) {
    mc_values vals = *(mc_values*)msg;
    char buf[100];
    snprintf(buf, sizeof(buf), "Current: %d\nRPM: %d\n", (int)vals.current_motor, (int)vals.rpm);
    Serial.println(buf);
    
  }
}



