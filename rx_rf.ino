#include<RH_ASK.h>
#include<SPI.h>

RH_ASK rf_driver;

void setup() {
  // put your setup code here, to run once:

  rf_driver.init();
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t buf[11];
  uint8_t buflen = sizeof(buf);

  if(rf_driver.recv(buf,&buflen)){
    Serial.print("Message received: ");
    Serial.println((char *)buf);
  }


}
