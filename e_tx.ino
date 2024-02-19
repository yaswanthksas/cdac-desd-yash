#include <HardwareSerial.h>
HardwareSerial TxSerial(1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  TxSerial.begin(9600, SERIAL_8N1, 16, 17); // (Baud Rate, Protocol, Rx-pin, Tx-pin) }


}

void loop() {
  // put your main code here, to run repeatedly:
  TxSerial.flush();
  TxSerial.println("Helloworld "+String(millis()/1000));
  delay(1000);
}

