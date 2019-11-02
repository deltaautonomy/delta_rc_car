/**
  Delta Autonomy RC Car Firmware
  @author Heethesh Vhavle
  @version 1.0.0
  @date 31-OCT-2019
*/

#include "communication.h"

/************************** MACROS **************************/

#define DEFAULT_LED 13
#define LEDR 9
#define LEDG 10
#define LEDB 11
#define BLDC 3
#define SERVO 5

/************************** GLOBALS **************************/

// Colors
enum class Color { red, yellow, green, cyan, blue, magenta, on, off };

/************************** RGB LED **************************/

void set_rgb_digital(int r, int g, int b) {
  digitalWrite(LEDR, !r);
  digitalWrite(LEDG, !g);
  digitalWrite(LEDB, !b);
}

void set_rgb_analog(int r, int g, int b) {
  analogWrite(LEDR, 255 - r);
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}

void set_rgb_color(Color color) {
  switch (color) {
    case Color::red:
      set_rgb_digital(1, 0, 0); break;
    case Color::yellow:
      set_rgb_digital(1, 1, 0); break;
    case Color::green:
      set_rgb_digital(0, 1, 0); break;
    case Color::cyan:
      set_rgb_digital(0, 1, 1); break;
    case Color::blue:
      set_rgb_digital(0, 0, 1); break;
    case Color::magenta:
      set_rgb_digital(1, 0, 1); break;
    case Color::on:
      set_rgb_digital(1, 1, 1); break;
    case Color::off:
      set_rgb_digital(0, 0, 0); break;
  }
}

void rgb_init() {
  // Set pin mode
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Set default state
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

/************************* CONTROLS *************************/

void write_servo(uint8_t value) {
  // TODO: Implement slow servo sweep
  analogWrite(SERVO, value);
}

void write_bldc(uint8_t value) {
  // TODO: Implement slow sweep
  analogWrite(BLDC, value);
}

void actuator_init() {
  // Set pin mode
  pinMode(SERVO, OUTPUT);
  pinMode(BLDC, OUTPUT);

  // Set default state
  analogWrite(SERVO, 127);  // TODO: Calibrate this center angle
  analogWrite(BLDC, 0);  // TODO: Verify this default value
}

/************************* ROUTINES *************************/

void main_routine() {
  // Recieve data from joystick
  if (!recieve_data()) return;

  if (rx_packet.global_arm) {
    analogWrite(LEDR, 255 - rx_packet.throttle);
    analogWrite(LEDB, 255 - rx_packet.steering);
    digitalWrite(LEDG, !rx_packet.direction);
  } else {
    set_rgb_color(Color::off);
  }
}

void display() {
  Serial.print("ARM: "); Serial.print(rx_packet.global_arm);
  Serial.print(" | THR: "); Serial.print(rx_packet.throttle);
  Serial.print(" | DIR: "); Serial.print(rx_packet.direction);
  Serial.print(" | STR: "); Serial.println(rx_packet.steering);
}

/*************************** MAIN ***************************/

void setup() {
  rgb_init();
  actuator_init();
  pinMode(DEFAULT_LED, OUTPUT);
  Serial.begin(9600);
  Serial.println("\n**** Delta Autonomy RC Car Firmware ****\n");
}

void loop() {
  main_routine();
  // set_rgb_color(Color::yellow);
}
