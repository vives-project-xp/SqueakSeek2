// ESP32 (Atom Lite) -> Pololu 3pi (serial slave)  **MINIMAL BINARY TEST**
#include <Arduino.h>

static const int RX_PIN = 32;     // from 3pi TX
static const int TX_PIN = 26;     // to 3pi RX
static const long UART_BAUD = 115200;
static const long USB_BAUD  = 115200;

HardwareSerial RobotSerial(1);

// Map -255..255 -> 0..127 en kies richting cmd (C1/C2 voor M1, C5/C6 voor M2)
static inline uint8_t spd127(int v) {
  v = constrain(v, -255, 255);
  int mag = abs(v) * 127 / 255;        // 0..127
  return (uint8_t)mag;
}

void setMotors_3pi(int left, int right) {
  uint8_t fwd1 = 0xC1, back1 = 0xC2;   // M1
  uint8_t fwd2 = 0xC5, back2 = 0xC6;   // M2

  uint8_t buf[4];
  // M1 (left)
  buf[0] = (left >= 0) ? fwd1 : back1;
  buf[1] = spd127(left);
  // M2 (right)
  buf[2] = (right >= 0) ? fwd2 : back2;
  buf[3] = spd127(right);

  RobotSerial.write(buf, 2);           // stuur M1 cmd + data
  RobotSerial.write(buf+2, 2);         // stuur M2 cmd + data

  // debug: hexdump
  Serial.printf("TX: %02X %02X  %02X %02X\n", buf[0],buf[1],buf[2],buf[3]);
}

void setup() {
  Serial.begin(USB_BAUD);
  delay(150);
  RobotSerial.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("\n[3pi binary sanity] gebruikt 0xC1/C2 & 0xC5/C6 (geen CR/LF)");
}

uint32_t t0=0;
bool flip=false;

void loop() {
  if (millis() - t0 >= 1000) {
    t0 = millis();
    flip = !flip;
    int s = 160;                       // pas snelheid aan
    // wissel elke seconde: vooruit <-> achteruit
    setMotors_3pi(flip ? +s : -s, flip ? +s : -s);
  }

  while (RobotSerial.available()) Serial.write(RobotSerial.read());
}