// Original works by https://github.com/tolunaygul
// Modified by Peter at PT Motorsport AU
// Modified by Dylan Larsen
// This code suits the CAN_IO_Mini_V1.2 board from PT Motorsport AU with Arduino Nano and MCP2515 CAN controller
// This code is for the 1.2 Board

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rx_id;           // storage for can data
byte len = 0;            // storage for can data
byte rx_buf[8];           // storage for can data

#define CAN0_INT 2                // Set INT to pin 2
MCP_CAN can0(10);              // set CS pin to 10

#define LED_PIN A5                // Set CAN LED pin to analog pin 5

// Define a global variable for the base CAN address
unsigned int avi_send_can_address = 0x2C0;
unsigned int dpi_send_can_address_1 = 0x2C2;
unsigned int dpi_send_can_address_2 = 0x2C4;
unsigned int dpo_rx_can_address_1 = 0x2D0;
unsigned int dpo_rx_can_address_2 = 0x2D2;
unsigned int keep_alive_can_address = 0x2C6;

// Define the digital pins for the DPO outputs
constexpr int dpo1 = 3;
constexpr int dpo2 = 5;
constexpr int dpo3 = 6;
constexpr int dpo4 = 9;

//define the digital pins for the DPI inputs
constexpr int dpi1_input = 4;           // dpi 1 input
constexpr int dpi2_input = 7;           // dpi 2 input
constexpr int dpi3_input = 8;           // dpi 3 input
constexpr int dpi4_input = A4;          // dpi 4 input

bool dpi1_in = false;                   // storage for digital input value as bool
bool dpi2_in = false;                   // storage for digital input value as bool
bool dpi3_in = false;                   // storage for digital input value as bool
bool dpi4_in = false;                   // storage for digital input value as bool

unsigned int scaled_value_1 = 0;              // storage for 12 bit analog value
unsigned int scaled_value_2 = 0;              // storage for 12 bit analog value
unsigned int scaled_value_3 = 0;              // storage for 12 bit analog value
unsigned int scaled_value_4 = 0;              // storage for 12 bit analog value

unsigned long task1_interval = 100; // 100ms (10hz) interval for keep alive frame
unsigned long task2_interval = 20;  // 20ms (50hz) interval for analogue value sending
unsigned long task3_interval = 20;  // 20ms (50hz) interval for analogue value reading
unsigned long task4_interval = 20;  // 20ms (50hz) interval for reading digital input
unsigned long task1_millis = 0;     // storage for millis counter
unsigned long task2_millis = 0;     // storage for millis counter
unsigned long task3_millis = 0;     // storage for millis counter
unsigned long task4_millis = 0;     // storage for millis counter

byte dpo1_out = 0;                  // storage for DPO output 1
byte dpo2_out = 0;                  // storage for DPO output 2
byte dpo3_out = 0;                  // storage for DPO output 3
byte dpo4_out = 0;                  // storage for DPO output 4

void can_read() {
    if (rx_id == dpo_rx_can_address_1) {
        dpo1_out = rx_buf[0];
        dpo2_out = rx_buf[4];
    }

    if (rx_id == dpo_rx_can_address_2) {
        dpo3_out = rx_buf[0];
        dpo4_out = rx_buf[4];
    }
}

void drive_digital_pin() {
    if (dpo1_out < 0xFA) analogWrite(map(dpo1, 0x00, 0xF9, 0, 255), dpo1_out);          // drive analogue output if the received data is < 0xFA
    if (dpo1_out >= 0xFA) digitalWrite(dpo1, HIGH);                                    // drive the output high if value is 0xFA
    if (dpo1_out == 0x00) digitalWrite(dpo1, LOW);                                     // drive the output low if value is 0x00

    if (dpo2_out < 0xFA) analogWrite(map(dpo2, 0x00, 0xF9, 0, 255), dpo2_out);          // drive analogue output if the received data is < 0xFA
    if (dpo2_out >= 0xFA) digitalWrite(dpo2, HIGH);                                    // drive the output high if value is 0xFA
    if (dpo2_out == 0x00) digitalWrite(dpo2, LOW);                                     // drive the output low if value is 0x00

    if (dpo3_out < 0xFA) analogWrite(map(dpo3, 0x00, 0xF9, 0, 255), dpo3_out);          // drive analogue output if the received data is < 0xFA
    if (dpo3_out >= 0xFA) digitalWrite(dpo3, HIGH);                                    // drive the output high if value is 0xFA
    if (dpo3_out == 0x00) digitalWrite(dpo3, LOW);                                     // drive the output low if value is 0x00

    if (dpo4_out < 0xFA) analogWrite(map(dpo4, 0x00, 0xF9, 0, 255), dpo4_out);          // drive analogue output if the received data is < 0xFA
    if (dpo4_out >= 0xFA) digitalWrite(dpo4, HIGH);                                    // drive the output high if value is 0xFA
    if (dpo4_out == 0x00) digitalWrite(dpo4, LOW);                                     // drive the output low if value is 0x00
}

void send_keep_alive() {
    byte keep_alive[5] = { 0X10, 0x0A, 0x0A, 0x01, 0x00 };
    can0.sendMsgBuf(keep_alive_can_address, 0, 5, keep_alive);
}

void send_dpi_values()
{
    byte dpi_data_1[8];
    byte dpi_data_2[8];

    // Read the values from the digital pins
    dpi1_in = digitalRead(dpi1_input);
    dpi2_in = digitalRead(dpi2_input);
    dpi3_in = digitalRead(dpi3_input);
    dpi4_in = digitalRead(dpi4_input);

    //Serial.println(DPI1in);
    if (dpi1_in == 1) { dpi_data_1[0] = 250; } if (dpi1_in == 0) { dpi_data_1[0] = 0; }
    dpi_data_1[1] = 0;
    dpi_data_1[2] = 0;
    dpi_data_1[3] = 0;
    if (dpi2_in == 1) { dpi_data_1[4] = 250; } if (dpi2_in == 0) { dpi_data_1[4] = 0; }
    dpi_data_1[5] = 0;
    dpi_data_1[6] = 0;
    dpi_data_1[7] = 0;
    if (dpi3_in == 1) { dpi_data_2[0] = 250; } if (dpi3_in == 0) { dpi_data_2[0] = 0; }
    dpi_data_2[1] = 0;
    dpi_data_2[2] = 0;
    dpi_data_2[3] = 0;
    if (dpi4_in == 1) { dpi_data_2[4] = 250; } if (dpi4_in == 0) { dpi_data_2[4] = 0; }
    dpi_data_2[5] = 0;
    dpi_data_2[6] = 0;
    dpi_data_2[7] = 0;

    can0.sendMsgBuf(dpi_send_can_address_1, 0, 8, dpi_data_1);
    can0.sendMsgBuf(dpi_send_can_address_2, 0, 8, dpi_data_2);
}

void send_analog_values() {
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

    scaled_value_1 = map(analogRead(A0), 0, 1023, 0, 4095);  // read analogue value and scale to 12 bit
    scaled_value_2 = map(analogRead(A1), 0, 1023, 0, 4095);  // read analogue value and scale to 12 bit
    scaled_value_3 = map(analogRead(A2), 0, 1023, 0, 4095);  // read analogue value and scale to 12 bit
    scaled_value_4 = map(analogRead(A3), 0, 1023, 0, 4095);  // read analogue value and scale to 12 bit

    canMsg.m2C1.data.AVI1_V = scaled_value_1;  // add scaled analogue value to message
    canMsg.m2C1.data.AVI2_V = scaled_value_2;  // add scaled analogue value to message
    canMsg.m2C1.data.AVI3_V = scaled_value_3;  // add scaled analogue value to message
    canMsg.m2C1.data.AVI4_V = scaled_value_4;  // add scaled analogue value to message

    //constuct the can message with correct bytes
    byte send_analogue[8] = {
        canMsg.m2C1.bytes[1],
        canMsg.m2C1.bytes[0],
        canMsg.m2C1.bytes[3],
        canMsg.m2C1.bytes[2],
        canMsg.m2C1.bytes[5],
        canMsg.m2C1.bytes[4],
        canMsg.m2C1.bytes[7],
        canMsg.m2C1.bytes[6]
    };

    can0.sendMsgBuf(avi_send_can_address, 0, 8, send_analogue);  // send the can message onto the bus
}

void setup() {
    // start serial port an send a message with delay for starting
    Serial.begin(115200);
    Serial.println("analog reading to 12 bit test");
    delay(200);

    // initialize canbus with 1000kbit and 8mhz xtal
    if (can0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
        Serial.println("MCP2515 Initialized Successfully!");
    else
        Serial.println("Error Initializing MCP2515...");

    // Set operation mode to normal so the MCP2515 sends acks to received data.
    can0.setMode(MCP_NORMAL);

    pinMode(CAN0_INT, INPUT);      // set INT pin to be an input
    digitalWrite(CAN0_INT, HIGH);  // set INT pin high to enable interna pullup

    // Set the pin mode for DPOs
    pinMode(dpo1, OUTPUT);
    pinMode(dpo2, OUTPUT);
    pinMode(dpo3, OUTPUT);
    pinMode(dpo4, OUTPUT);

    // Set the pin mode for DPIs
    pinMode(dpi1_input, INPUT_PULLUP); // Remove "_PULLUP" if you want to use external pullup resistors, pull up to 5v only
    pinMode(dpi2_input, INPUT_PULLUP);
    pinMode(dpi3_input, INPUT_PULLUP);
    pinMode(dpi4_input, INPUT_PULLUP);

    // IO Box A or B Mode (A7 is analog only, hence the analog read)
    pinMode(A7, INPUT_PULLUP); // Set the analog pin 7 as input
    int analogValue = analogRead(A7); // Read the analog value from pin A7

    // Compare the analog value to a threshold
    if (analogValue < 512) { // Adjust the threshold as necessary
        avi_send_can_address++;
        dpi_send_can_address_1++;
        dpi_send_can_address_2++;
        dpo_rx_can_address_1++;
        dpo_rx_can_address_2++;
        keep_alive_can_address++;
    }

    pinMode(LED_PIN, OUTPUT);
    Serial.println("All OK");  // all ready to go !
}

void loop() {
    unsigned long current_millis = millis();  // Get current time in milliseconds

    // Execute task 1 every 1 second
    if (current_millis - task1_millis >= task1_interval) {
        task1_millis = current_millis;
        send_keep_alive();
    }

    // Execute task 2 every 5 seconds
    if (current_millis - task2_millis >= task2_interval) {
        task2_millis = current_millis;
        send_analog_values();
    }

    // Execute task 3 every 3 seconds
    if (current_millis - task3_millis >= task3_interval) {
        task3_millis = current_millis;
        drive_digital_pin();
    }

    // Execute task 4 every 4 seconds
    if (current_millis - task4_millis >= task4_interval) {
        task4_millis = current_millis;
        send_dpi_values();
    }

    // read can buffer when interrupted and jump to canread for processing.
    if (!digitalRead(CAN0_INT))  // If CAN0_INT pin is low, read receive buffer
    {
        can0.readMsgBuf(&rx_id, &len, rx_buf);  // Read data: len = data length, buf = data byte(s)
        can_read();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED state
    }
}
