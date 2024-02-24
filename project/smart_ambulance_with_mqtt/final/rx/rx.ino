#include <RH_ASK.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Define the receiver pin
#define RECEIVER_PIN 2

const char* ssid = "yaswanth";
const char* password = "123456789";
const char* mqtt_server = "192.168.238.15";

WiFiClient espClient;
PubSubClient client(espClient);

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

int value = 0;

#define grn 13
#define yel 3
#define red 4

#define grn2 7
#define yel2 8
#define red2 0

#define grn3 25
#define red3 5
#define yel3 26 

#define grn4 12
#define red4 18
#define yel4 9 

int grn_delay = 5000;
int yel_delay = 2000;
int red_delay = 5000;


void green_light() {
  digitalWrite(grn, HIGH);
  digitalWrite(yel, LOW);
  digitalWrite(red, LOW);
}

void yellow_light() {
  digitalWrite(grn, LOW);
  digitalWrite(yel, HIGH);
  digitalWrite(red, LOW);
}

void red_light() {
  digitalWrite(grn, LOW);
  digitalWrite(yel, LOW);
  digitalWrite(red, HIGH);
}

void green_light2() {
  digitalWrite(grn2, HIGH);
  digitalWrite(yel2, LOW);
  digitalWrite(red2, LOW);
}

void yellow_light2() {
  digitalWrite(grn2, LOW);
  digitalWrite(yel2, HIGH);
  digitalWrite(red2, LOW);
}

void red_light2() {
  digitalWrite(grn2, LOW);
  digitalWrite(yel2, LOW);
  digitalWrite(red2, HIGH);
}

void green_light3() {
  digitalWrite(grn3, HIGH);
  digitalWrite(yel3, LOW);
  digitalWrite(red3, LOW);
}

void yellow_light3() {
  digitalWrite(grn3, LOW);
  digitalWrite(yel3, HIGH);
  digitalWrite(red3, LOW);
}

void red_light3() {
  digitalWrite(grn3, LOW);
  digitalWrite(yel3, LOW);
  digitalWrite(red3, HIGH);
}

void green_light4() {
  digitalWrite(grn4, HIGH);
  digitalWrite(yel4, LOW);
  digitalWrite(red4, LOW);
}

void yellow_light4() {
  digitalWrite(grn4, LOW);
  digitalWrite(yel4, HIGH);
  digitalWrite(red4, LOW);
}

void red_light4() {
  digitalWrite(grn4, LOW);
  digitalWrite(yel4, LOW);
  digitalWrite(red4, HIGH);
}

// Create an instance of the RF driver
RH_ASK rf_driver(2000, RECEIVER_PIN);


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  //setting to wifi_sta mode 
  WiFi.begin(ssid, password);
  //connection of microcontroller to wifi 

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  if((char)payload[0]== 'R' && (char)payload[1]=='e' && (char)payload[2] == 'd'){
        Serial.print("Red on lane 2");
        Serial.print("Red on Lane 3");
        Serial.print("Red on lane 4");
        // digitalWrite(red2,HIGH);
        // digitalWrite(red3,HIGH);
        // digitalWrite(red4,HIGH);
        red_light2();
        red_light3();
        red_light4();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    if (client.connect("CL_ID_3", "yesh", "yesh@123")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "Green");

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

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  setup_wifi();   // connecting mc to wifi 
  client.setServer(mqtt_server, 1883);  //set the mqtt server connections 
  client.setCallback(callback);    //It sets the callback function to handle incoming messages. Whenever a message is received from the MQTT broker, this function (callback) will be invoked automatically to process the message.

  // Initialize RF driver
  if (!rf_driver.init()) {
    Serial.println("RF driver init failed");
  }

}

void loop() {
  if (!client.connected()) {      //if mqtt client not connected to mqtt broker 
     reconnect();         //reconnect it with broker
  }
  client.loop();   // client to perform background tasks 
  //mataining mqtt connection 


  // uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buf[19];
  uint8_t buflen = sizeof(buf);

  // Check if a message is available
  if (rf_driver.recv(buf, &buflen)) {
    // Message received
    buf[buflen] = '\0';  // Null-terminate the string
    Serial.print("Message received: ");
    Serial.println((char*)buf);
    green_light();
    ++value;
    snprintf(msg, MSG_BUFFER_SIZE, "Green Alert #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("outTopic", msg);

    delay(grn_delay);

  } else {
    green_light();
    delay(grn_delay);
    yellow_light();
    delay(yel_delay);
    red_light();
    delay(red_delay);
  }
}
// this code for receiver
