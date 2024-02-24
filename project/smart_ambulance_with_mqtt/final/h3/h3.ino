#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <TinyGPS++.h>


// Update these with values suitable for your network and MQTT broker
const char* ssid = "yash";
const char* password = "12345678";
const char* mqtt_server = "192.168.238.15";
const char* mqtt_user = "yesh";
const char* mqtt_password = "yesh@123";

#define LM75_ADDRESS 0x48  // LM75 I2C address
#define bright 23 

#define MSG_BUFFER_SIZE (50)
#define GPS_BAUDRATE 9600   // The default baudrate of NEO-6M is 9600
char msg[MSG_BUFFER_SIZE];  //string to publish the data
char bpm[MSG_BUFFER_SIZE];
char avg_bpm[MSG_BUFFER_SIZE];
char oxy_spo2[MSG_BUFFER_SIZE];
char temp[MSG_BUFFER_SIZE];

const byte RATE_SIZE = 4;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;     //Time at which the last beat occurred
float beatsPerMinute;  //heart rate
int beatAvg;           //average of four heartrates
const int buzzer = 2;
#define core 1

WiFiClient espClient;
PubSubClient client(espClient);
MAX30105 particleSensor;
TinyGPSPlus gps;  // the TinyGPS++ object

// void Buzzer_ALARM();
void setup_wifi();
float readTemperature();
// void BuzzerTone(int pin, int frequency, int duration);

//   We start by connecting to a WiFi network
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


//   to subscribe to mqtt broker
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  //  if ((char)payload[0] == '1') {
  //   digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
  //   // but actually the LED is on; this is because
  //   // it is active low on the ESP-01)
  // } else {
  //   digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  // }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("CL_ID_1", mqtt_user, mqtt_password)) {
      // to establishing connection between esp32 and mqtt broker
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("irValue", "hello world");
      snprintf(bpm, MSG_BUFFER_SIZE, "%f", beatsPerMinute);
      client.publish("heart rate", bpm);
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float readTemperature() {
  Wire.beginTransmission(LM75_ADDRESS);  // Start communication with LM75 at specified address
  Wire.write(0x00);  // Select LM75 register for temperature reading
  Wire.endTransmission();  // End the transmission

  delay(100);  // Wait for the LM75 to complete the temperature conversion

  Wire.requestFrom(LM75_ADDRESS, 2);  // Request 2 bytes of data from LM75
  while (Wire.available() < 2);  // Wait until data is available

  int16_t rawTemperature = Wire.read() << 8 | Wire.read();  // Combine two bytes into a 16-bit raw temperature value
  float temperature = rawTemperature / 256.0;  // Convert raw temperature to Celsius
  // The LM75 has a resolution of 0.5°C per bit, and dividing by 256 gives the temperature in degrees Celsius.
  return temperature;  // Return the temperature value
}

// void Buzzer_ALARM()
// {
//   int i=0;
//   while(i<25)
//   {
//     BuzzerTone(buzzer, 1000, 500); // Frequency: 1000 Hz, Duration: 500 ms
//     delay(1000);
//   }
// }

// void BuzzerTone(int pin, int frequency, int duration)
// {
//   tone(pin, frequency);
//   delay(duration);
//   noTone(pin);
// }


TaskHandle_t heartBeatTaskHandle, oxygenRateTaskHandle, temperatureTaskHandle, gpsTaskHandle;

void heartBeatTask(void* data) {
  while (1) {
    if (!client.connected()) {  //mqtt client not currently connected
      reconnect();              //reconnect broker
    }
    client.loop();

    long irValue = particleSensor.getIR();
    uint32_t redValue = particleSensor.getRed();

    if (checkForBeat(irValue) == true)  // Perform analysis on the IR signal to detect a beat // compare the current IR value with a threshold // If the signal crosses the threshold, consider it a beat
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;  //delta - difference between two heart beats. current time - last time of heart beat
      lastBeat = millis();               //number of milliseconds since the Arduino started running the current program.

      beatsPerMinute = 60 / (delta / 1000.0);  //(delta/1000.0) convert to seconds
      //This expression calculates the number of beats per minute.
      //The formula for BPM is typically expressed as "beats per minute = 60 / (time in seconds between beats)." Therefore, dividing 60 by the time in seconds between beats gives the BPM.

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable
        //after storing again make ratespot to zero 4%4 is 0

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    if(beatAvg < 50){
      digitalWrite(buzzer,HIGH);
      // Buzzer_ALARM();
    }
    else{
    digitalWrite(buzzer,LOW);
    }



    snprintf(bpm, MSG_BUFFER_SIZE, "Beatsperminute #%lf", beatsPerMinute);
    Serial.print("Publish message: ");
    Serial.println(bpm);
    client.publish("heart rate", bpm);

    snprintf(avg_bpm, MSG_BUFFER_SIZE, "average_beat #%lf", beatAvg);
    Serial.print("Publish message: ");
    Serial.println(avg_bpm);
    client.publish("average heart rate", avg_bpm);


    snprintf(msg, MSG_BUFFER_SIZE, "irvalue #%ld", irValue);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("irValue", msg);

    vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 500 milliseconds
  }
  vTaskDelete(NULL);
}

//oxygen task 
void oxygenRateTask(void *data){
  while(1){
    if (!client.connected()) {  //mqtt client not currently connected
      reconnect();              //reconnect broker
    }
    client.loop();

    long irValue = particleSensor.getIR();
    uint32_t redValue = particleSensor.getRed();

    float ratio = static_cast<float>(redValue) / static_cast<float>(irValue);
    float spo2 = -45.060 * pow(ratio, 2) + 30.354 * ratio + 94.845;

    snprintf (oxy_spo2, MSG_BUFFER_SIZE, "oxygen percent #%lf %", spo2);
    Serial.print("Publish message: ");
    Serial.println(oxy_spo2);
    client.publish("oxygen percentage",oxy_spo2);

    if(spo2 < 60){
      for(int x=0;x<255;x++){
          analogWrite(23,x);
          delay(1);
      }
      for(int x=255;x>0;x--){
          analogWrite(23,x);
          delay(1);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 500 milliseconds
  }
  vTaskDelete(NULL);
}

void temperatureTask(void *data){
  while(1){
    if (!client.connected()) {  //mqtt client not currently connected
      reconnect();              //reconnect broker
    }
    client.loop();

    //temperature readings 
    // float temperatureF = particleSensor.readTemperatureF();

    float temperature = readTemperature();  // Call the function to read temperature


    snprintf (temp, MSG_BUFFER_SIZE, "temperature #%lf %", temperature);
    Serial.print("Publish message: ");
    Serial.println(temp);
    client.publish("temperature ",temp);

    vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 500 milliseconds
  }
  vTaskDelete(NULL);
}

void gpsTask(void* data) {
  while (1) {
    if (Serial2.available() > 0) {
      if (gps.encode(Serial2.read())) {
        if (gps.location.isValid()) {
          Serial.print("- latitude: ");
          Serial.println(gps.location.lat());

          Serial.print("- longitude: ");
          Serial.println(gps.location.lng());

          Serial.print("- altitude: ");
          if (gps.altitude.isValid())
            Serial.println(gps.altitude.meters());
          else
            Serial.println("INVALID");
        } else {
          Serial.println("- location: INVALID");
        }

        Serial.print("- speed: ");
        if (gps.speed.isValid()) {
          Serial.print(gps.speed.kmph());
          Serial.println(" km/h");
        } else {
          Serial.println("INVALID");
        }

        Serial.print(F("- GPS date&time: "));
        if (gps.date.isValid() && gps.time.isValid()) {
          Serial.print(gps.date.year());
          Serial.print(F("-"));
          Serial.print(gps.date.month());
          Serial.print(F("-"));
          Serial.print(gps.date.day());
          Serial.print(F(" "));
          Serial.print(gps.time.hour());
          Serial.print(F(":"));
          Serial.print(gps.time.minute());
          Serial.print(F(":"));
          Serial.println(gps.time.second());
        } else {
          Serial.println(F("INVALID"));
        }

        Serial.println();
      }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));

    vTaskDelay(pdMS_TO_TICKS(5000));  // Delay for 500 milliseconds
    
  }
  vTaskDelete(NULL);
}


void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(bright,OUTPUT);
  Serial.begin(115200);
  setup_wifi();                         // connecting mc to wifi
  client.setServer(mqtt_server, 1883);  // set the broker details
  client.setCallback(callback);         //It sets the callback function to handle incoming messages. Whenever a message is received from the MQTT broker, this function (callback) will be invoked automatically to process the message.

  Serial.println("Initializing...");                // Initialize max30102 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  //If the sensor initialization fails, the code enters an infinite loop (while(1)) to halt the program execution. This is a way to stop further execution, as the sensor is not functioning as expected.
  Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup();
  Wire.begin();          // Initialize the I2C communication


  //Task Creations
  //part1
  xTaskCreatePinnedToCore(heartBeatTask, "heartBeatTask", 2048, NULL, 6, &heartBeatTaskHandle, core);
  xTaskCreatePinnedToCore(oxygenRateTask, "oxygenRateTask", 2048, NULL, 6, &oxygenRateTaskHandle, core);
  xTaskCreatePinnedToCore(temperatureTask, "temperatureTask", 2048, NULL, 6, &temperatureTaskHandle, core);
  xTaskCreatePinnedToCore(gpsTask, "gpsTask", 2048, NULL, 6, &gpsTaskHandle, core);
}

void loop() {
    
}
