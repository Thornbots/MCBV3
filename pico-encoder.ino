#include "hardware/pio.h"
#include "PicoEncoder.h"
#include <Wire.h>

const byte PICO_I2C_ADDRESS = 0x55;
const byte PICO_I2C_SDA = 16;
const byte PICO_I2C_SCL = 17;
const byte PICO_LED = 25;


byte volatile rx_flag = 0;
byte volatile tx_flag = 0;
uint8_t volatile ram_addr = 0;
uint8_t volatile ram[256];

TwoWire wire(PICO_I2C_SDA, PICO_I2C_SCL);


PicoEncoder encoder1;
PicoEncoder encoder2;

const float radius = .0175;
const float cpr = 4096;
const float scale = radius * 2 * 3.1415926f / (64 * cpr);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PICO_LED, OUTPUT); //Built-in LED

  const int encoder1_pinA = p12;
  const int encoder1_pinB = p13;
  const int encoder2_pinA = p14;
  const int encoder2_pinB = p15;


  encoder1.begin(encoder1_pinA);
  encoder2.begin(encoder2_pinA);
  analogReadResolution(12);

  wire.begin(PICO_I2C_ADDRESS);    // join i2c bus as slave
  wire.onReceive(i2c_receive);     // i2c interrupt receive
  wire.onRequest(i2c_transmit);    // i2c interrupt send

  digitalWrite(PICO_LED, HIGH);
  
}
  static int microlast;
  static int dt;
  static float turretpos;
  static float turretposlast;
  static float turretvel;

void loop() {

  // dt = micros() - microlast;
  // if (dt >= 1998) { //2khz
  //   microlast = micros(); 
  encoder1.update();
  encoder2.update();
  //   Serial.print(" ======");
  // Serial.print(encoder1.position * scale);
  //   Serial.print(" ======");
  // Serial.print(encoder2.position * scale);
  //   Serial.print(" ======");
  // Serial.print(encoder1.speed * scale);
  //   Serial.print(" ======");
  // Serial.print(encoder2.speed * scale);
  //   Serial.print(" ======");
  // Serial.println(turretvel);
  // Serial.println(dt);
  // Serial.println(microlast);
  
  // }

  // if (rx_flag) {
  //   rx_flag = 0;
  //   digitalWrite (PICO_LED, HIGH);
  //   delay(1);
  //   digitalWrite (PICO_LED, LOW);
  // }

  // if (tx_flag) {
  //   tx_flag = 0;
  //   digitalWrite (PICO_LED, HIGH);
  //   delay(1);
  //   digitalWrite (PICO_LED, LOW);
  // }
}

void i2c_receive(int bytes_count) {     // bytes_count gives number of bytes in rx buffer   
  if (bytes_count == 0) {
    return;
  }
  ram_addr = (uint8_t)wire.read();      // first byte is ram offset address (0-255)
  // for (byte i=1; i<bytes_count; i++) {
  //   ram[ram_addr] = (uint8_t)wire.read();
  //   ram_addr++;
  // } 
  // rx_flag = bytes_count;
  Serial.print("rx:");
  Serial.print(ram_addr);
  Serial.print("\n");

}

void i2c_transmit() {
  // tx_flag = 1;
  // wire.write(69);
  // wire.write(70);
  // wire.write(71);
  // wire.write(73);

  float data[4] = {encoder1.position * scale,encoder2.position * scale,encoder1.speed * scale,encoder2.speed * scale};
  wire.write((uint8_t*)data, sizeof(float)*4);

  Serial.print(encoder1.position * scale);
  Serial.print("    ");
  Serial.print(encoder2.position * scale);
  Serial.print("    ");
  Serial.print(encoder1.speed * scale);
  Serial.print("    ");
  Serial.print(encoder2.speed * scale);
  Serial.println("");

  // ram_addr++;
  // Serial.print("tx\n");
}
