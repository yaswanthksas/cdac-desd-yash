#include <RH_ASK.h>
#include <SPI.h>
#include <RadioHead.h>

// Define the transmitter pin
#define TRANSMITTER_PIN 4

// Create an instance of the RF driver
RH_ASK rf_driver(2000, TRANSMITTER_PIN);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize RF driver
  if (!rf_driver.init()) {
    Serial.println("RF driver init failed");
  }
}
void loop() {
  const char *message = "Ambulance Arriving";
  uint8_t buf[strlen(message)];

  // Copy the message into the buffer
  strcpy((char *)buf, message);

  // Send the message
  rf_driver.send((uint8_t *)buf, strlen(message));
  rf_driver.waitPacketSent();

  // Print the message sent
  Serial.println("Message sent: ");
  // Serial.println(message);

  // Wait before sending the next message
  delay(1000);
}