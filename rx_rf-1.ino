#include <Arduino.h>
#include <RH_ASK.h>
#include <SPI.h>

// Set the RF transmitter module data pin
#define RF_TX_PIN 17

// Create an instance of the RH_ASK driver
RH_ASK rf_driver;

void setup() {
  Serial.begin(115200);
  
  // Initialize the RF driver
  if (!rf_driver.init()) {
    Serial.println("RF driver initialization failed. Check wiring and power.");
    while (1);  // Infinite loop to halt the program in case of failure
  }

  // Print a message to the serial monitor
  Serial.println("Ready to transmit...");
}

void loop() {
  const char *message = "Hello, RF World!"; // Message to be transmitted

  // Send the message
  rf_driver.send((uint8_t *)message, strlen(message));

  // Wait for the transmission to complete
  rf_driver.waitPacketSent();

  // Print a message to the serial monitor
  Serial.println("Message transmitted!");

  // Delay before sending the next message
  delay(5000); // Adjust the delay based on your requirements
}
