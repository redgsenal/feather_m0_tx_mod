// Feather9x_TX Modification
// Modification from Feather9x_TX example
// -*- mode: C++ -*-
// sends 3 data representing 3 different buttons

#include <SPI.h>
#include <RH_RF95.h>

// definition for Feather M0 w/Radio example ADAFRUIT_FEATHER_M0 or ADAFRUIT_FEATHER_M0_EXPRESS or ARDUINO_SAMD_FEATHER_M0. 
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4

#define LED_RED 14
#define LED_YELLOW 15
#define LED_GREEN 16

#define BTN_RED 17
#define BTN_YELLOW 18
#define BTN_GREEN 19

// Change frequency to match RX's freq
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int btn_red_state;
int btn_yellow_state;
int btn_green_state;

void setup() {
  // setup LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // setup BUTTONs
  pinMode(BTN_RED, INPUT);
  pinMode(BTN_YELLOW, INPUT);
  pinMode(BTN_GREEN, INPUT);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void press_led_pin(int led_pin, int btn_state){
  digitalWrite(led_pin, LOW);
  if (btn_state == HIGH){
    digitalWrite(led_pin, HIGH);
  }  
}

void loop() {

  // get the button states and press LED on
  press_led_pin(LED_RED, digitalRead(BTN_RED));
  press_led_pin(LED_YELLOW, digitalRead(BTN_YELLOW));
  press_led_pin(LED_GREEN, digitalRead(BTN_GREEN));

  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server

  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }

}
