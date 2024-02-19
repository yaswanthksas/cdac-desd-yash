#include <HardwareSerial.h>
HardwareSerial RxSerial(1);
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
  RxSerial.begin(9600, SERIAL_8N1, 16, 17); 

}

void loop() {
  // put your main code here, to run repeatedly:
  String received = "";
  while(RxSerial.available()){
    received = (char)(RxSerial.read());
    Serial.print(received);
  }

}
