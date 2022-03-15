#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>
#include "DHT.h"
#define SensorPin A0 
#define DHTPIN 7     // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11 
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
float sensorValue = 0; 
///////please enter your SSID and Password
char ssid[] = "POCO X3 Pro";        // your network SSID (name)
char pass[] = "1234567890";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// MQTT server setup
const char broker[] = "155.4.118.139"; 
int        port     = 1883;
const char topic[]  = "P18";

//Test varaibles

String PoParam="";
String DID;
float Xaxis, Yaxis, Zaxis;
const long interval = 100;
unsigned long Millis = 0;
String subMessage = "";
String submotor="";
boolean stat=0;
String subString ="Led is OFF";
String submess ="Plant is good";

float t, h, x, y, z;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  Serial.begin(9600); 
  Serial.println("DHTxx test!");
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  dht.begin();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
    
  }

    while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  }

  // subscribe to a topic
  mqttClient.subscribe(topic);

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

   if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print(IMU.accelerationSampleRate());

}

void loop() {
  StaticJsonDocument<500> OutMes;
  StaticJsonDocument<400> inMes;

    IMU.readAcceleration(x, y, z);
  
//Data Serialization
    OutMes["Temperature"]=(t);
    OutMes["Soil_Moisture"]=(sensorValue);
    OutMes["Motor_Status"]=(submotor);
    OutMes["humidity"]=(h);
    //OutMes["LedStatus"]=(subString);
    OutMes["Plant_Condition"]=(submess);
   serializeJson(OutMes, PoParam);
   Serial.println(PoParam);

   //Read incoming message
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    subMessage = "";
   
    // use the Stream interface to print the contents
    while (mqttClient.available()) {
      subMessage = subMessage + (char)mqttClient.read();
    }

    //Taking Action acording to subscribed message
    if(subMessage == "ONOFF") {
      if(stat==1){
        digitalWrite(13, LOW);
        digitalWrite(LED_BUILTIN, LOW);
        stat=0;
        subString = "Led is OFF";
        }
        else if(stat==0){
           digitalWrite(13, HIGH);
           digitalWrite(LED_BUILTIN, HIGH);
           stat=1;
           subString = "Led is ON";
          }
     
    } 
  }

  //Publishing measured data through MQTT
   if(millis()-Millis>interval){
       mqttClient.beginMessage(topic);
        mqttClient.print(PoParam);
        mqttClient.endMessage();
        Millis=millis();
  }

  delay(10);
  PoParam="";
  // Wait a few seconds between measurements.
  delay(2500);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius
  t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  int val = analogRead(A0);
  if ((val <= 1000)&&(sensorValue>=750)&& stat!=0) {
    digitalWrite(13, HIGH);
    submotor=("Motor is ON");
    } 
    else {
      digitalWrite(13, LOW);
      submotor=("Motor is OFF");
      }   
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  for (int i = 0; i <= 100; i++) 
 { 
   sensorValue = sensorValue + analogRead(SensorPin); 
   delay(1); 
 } 
 sensorValue = sensorValue/100.0; 
 delay(30); 
 if (sensorValue <=800){
  submess="b";
 }
 else{
  submess="a";
 }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  float hi = dht.computeHeatIndex(f, h);
 
}
