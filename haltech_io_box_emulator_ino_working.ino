#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;           // storage for can data
unsigned char len = 0;            // storage for can data
unsigned char rxBuf[8];           // storage for can data

#define CAN0_INT 2                // Set INT to pin 2
MCP_CAN CAN0(10);                 // set CS pin to 10r

const int DPO1 = 3;
const int DPO2 = 5;
const int DPO3 = 6;
const int DPO4 = 9;

int DPI1input = 4; // dpi 1 input
int DPI2input = 4; // dpi 2 input
int DPI3input = 4; // dpi 3 input
int DPI4input = 4; // dpi 4 input

bool DPI1in = 0;
bool DPI2in = 0;
bool DPI3in = 0;
bool DPI4in = 0;

int scaledvalue1 = 0;             // storage for 12 bit analog value
int scaledvalue2 = 0;             // storage for 12 bit analog value
int scaledvalue3 = 0;             // storage for 12 bit analog value
int scaledvalue4 = 0;             // storage for 12 bit analog value

unsigned long task1Interval = 100; // 100ms interval for keep alive frame
unsigned long task2Interval = 20;  // 20ms interval for analogue value sending
unsigned long task3Interval = 20;  // 20ms interval for analogue value reading 
unsigned long task4Interval = 20;  // 20ms interval for reading digital input
unsigned long task1Millis = 0;     // storage for millis counter
unsigned long task2Millis = 0;     // storage for millis counter
unsigned long task3Millis = 0;     // storage for millis counter
unsigned long task4Millis = 0;     // storage for millis counter

byte DPO1out = 0;                 // storage for DPO output 1
byte DPO2out = 0;                 // storage for DPO output 2
byte DPO3out = 0;                 // storage for DPO output 3
byte DPO4out = 0;                 // storage for DPO output 4

void setup() {
  // start serial port an send a message with delay for starting
  Serial.begin(115200);   
  Serial.println("analog reading to 12 bit test");
  delay(1000);

  // initialize canbus with 1000kbit and 16mhz xtal
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) 
  Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);  

  pinMode(CAN0_INT, INPUT);      // set INT pin to be an input
  digitalWrite(CAN0_INT, HIGH);  // set INT pin high to enable interna pullup


  pinMode(DPO1, OUTPUT); // set pin number for DPO1
  pinMode(DPO2, OUTPUT); // set pin number for DPO2
  pinMode(DPO3, OUTPUT); // set pin number for DPO3
  pinMode(DPO4, OUTPUT); // set pin number for DPO4

  pinMode(DPI1input, INPUT);
  digitalWrite(DPI1input, HIGH);
  
  
  Serial.println("All OK");  // all ready to go !
}




void loop() {

  unsigned long currentMillis = millis();  // Get current time in milliseconds

  // Execute task 1 every 1 second
  if (currentMillis - task1Millis >= task1Interval) {
    task1Millis = currentMillis;
    SendKeepAlive();
  }

  // Execute task 2 every 5 seconds
  if (currentMillis - task2Millis >= task2Interval) {
    task2Millis = currentMillis;
    SendAnalogValues();
  }

  // Execute task 3 every 3 seconds
  if (currentMillis - task3Millis >= task3Interval) {
    task3Millis = currentMillis;
    DriveDigitalPin();
  }

  // Execute task 4 every 4 seconds
  if (currentMillis - task4Millis >= task4Interval) {
    task4Millis = currentMillis;
    SendDPIValues();
  }

  // read can buffer when interrupted and jump to canread for processing.
  if (!digitalRead(CAN0_INT))  // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Read data: len = data length, buf = data byte(s)
    canRead();
  }
}


void canRead() {
  if (rxId == 0X2D1) {
    DPO1out = rxBuf[0];
    DPO2out = rxBuf[4];
  }

  if (rxId == 0X2D3) {
    DPO3out = rxBuf[0];
    DPO4out = rxBuf[4];
  }
}

void DriveDigitalPin() {
    if (DPO1out < 0xFA) {analogWrite(map(DPO1, 0x00, 0xF9, 0, 255), DPO1out); }          //drive analogue output if the received data is < 0xFA
    if (DPO1out == 0xFA) {digitalWrite(DPO1, HIGH); }                                    //drive the output high if value is 0xFA
    if (DPO1out == 0x00) {digitalWrite(DPO1, LOW); }                                     //drive the output low if value is 0x00

    if (DPO2out < 0xFA) {analogWrite(map(DPO2, 0x00, 0xF9, 0, 255), DPO2out); }          //drive analogue output if the received data is < 0xFA
    if (DPO2out == 0xFA) {digitalWrite(DPO2, HIGH); }                                    //drive the output high if value is 0xFA
    if (DPO2out == 0x00) {digitalWrite(DPO2, LOW); }                                     //drive the output low if value is 0x00
  
    if (DPO3out < 0xFA) {analogWrite(map(DPO3, 0x00, 0xF9, 0, 255), DPO3out); }          //drive analogue output if the received data is < 0xFA
    if (DPO3out == 0xFA) {digitalWrite(DPO3, HIGH); }                                    //drive the output high if value is 0xFA
    if (DPO3out == 0x00) {digitalWrite(DPO3, LOW); }                                     //drive the output low if value is 0x00

    if (DPO4out < 0xFA) {analogWrite(map(DPO4, 0x00, 0xF9, 0, 255), DPO4out); }          //drive analogue output if the received data is < 0xFA
    if (DPO4out == 0xFA) {digitalWrite(DPO4, HIGH); }                                    //drive the output high if value is 0xFA
    if (DPO4out == 0x00) {digitalWrite(DPO4, LOW); }                                     //drive the output low if value is 0x00
}



void SendKeepAlive() {
  byte KeepAlive[5] = { 0X10, 0x09, 0x0a, 0x00, 0x00 };
  CAN0.sendMsgBuf(0x2C7, 0, 5, KeepAlive);
}


void SendDPIValues()
{
byte DPIdata1[8];
byte DPIdata2[8];
DPI1in = DPI1input;
DPI2in = DPI2input;
DPI3in = DPI3input;
DPI4in = DPI4input;

//Serial.println(DPI1in);
if (DPI1in == 1) { DPIdata1[0] = 250; } if (DPI1in == 0) { DPIdata1[0] = 0; } 
DPIdata1[1] = 0;
DPIdata1[2] = 0;
DPIdata1[3] = 0;
if (DPI2in == 1) { DPIdata1[4] = 250; } if (DPI2in == 0) { DPIdata1[4] = 0; } 
DPIdata1[5] = 0;
DPIdata1[6] = 0;
DPIdata1[7] = 0;

if (DPI3in == 1) { DPIdata2[0] = 250; } if (DPI3in == 0) { DPIdata2[0] = 0; } 
DPIdata2[1] = 0;
DPIdata2[2] = 0;
DPIdata2[3] = 0;
if (DPI4in == 1) { DPIdata2[4] = 250; } if (DPI4in == 0) { DPIdata2[4] = 0; } 
DPIdata2[5] = 0;
DPIdata2[6] = 0;
DPIdata2[7] = 0;


CAN0.sendMsgBuf(0x2C3, 0, 8, DPIdata1);
CAN0.sendMsgBuf(0x2C5, 0, 8, DPIdata2);

}


void SendAnalogValues() {

  // struct the analogue values
  struct m2C1truct {
    unsigned int AVI1_V : 16;  //0:3-1:0
    unsigned int AVI2_V : 16;  //2:3-3:0
    unsigned int AVI3_V : 16;  //4:3-5:0
    unsigned int AVI4_V : 16;  //6:3-7:0
  };

  // union / make a struct
  union union_m2C1 {
    struct m2C1truct data;
    byte bytes[8];
  };

  // construct the can message
  struct canMsg {
    union_m2C1 m2C1;
  } canMsg;


  scaledvalue1 = map(analogRead(A0), 1023, 0, 0, 4095);  // read analogue value and scale to 12 bit
  scaledvalue2 = map(analogRead(A1), 1023, 0, 0, 4095);  // read analogue value and scale to 12 bit
  scaledvalue3 = map(analogRead(A2), 1023, 0, 0, 4095);  // read analogue value and scale to 12 bit
  scaledvalue4 = map(analogRead(A3), 1023, 0, 0, 4095);  // read analogue value and scale to 12 bit

  canMsg.m2C1.data.AVI1_V = scaledvalue1;  // add scaled analogue value to message
  canMsg.m2C1.data.AVI2_V = scaledvalue2;  // add scaled analogue value to message
  canMsg.m2C1.data.AVI3_V = scaledvalue3;  // add scaled analogue value to message
  canMsg.m2C1.data.AVI4_V = scaledvalue4;  // add scaled analogue value to message


  //constuct the can message with correct bytes
  byte SendAnalogue[8] = {
    canMsg.m2C1.bytes[1],
    canMsg.m2C1.bytes[0],
    canMsg.m2C1.bytes[3],
    canMsg.m2C1.bytes[2],
    canMsg.m2C1.bytes[5],
    canMsg.m2C1.bytes[4],
    canMsg.m2C1.bytes[7],
    canMsg.m2C1.bytes[6]
  };

  CAN0.sendMsgBuf(0x2C1, 0, 8, SendAnalogue);  // send the can message onto the bus
}