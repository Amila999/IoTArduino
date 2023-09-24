#include <WiFiNINA.h>
#include "secrets.h"
#include <PIDandLowPass.h>
#include <ThingSpeak.h>

WiFiClient  client;

//Simulation
float Ts = 0.1;
int count = 0;
float wait = 20; // Wait 20 seconds to update the channel again

//Controller
float u = 0;

float y_init = 25.0;
float y = y_init;

const int AirHeaterPin = 9;
const int temp_sensPin = 0;

//Constructor
PIDandLowPass pidAndLowPass(y_init);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pidAndLowPass.r = 35; //Setpoint
  pidAndLowPass.Kp = 0.35;
  pidAndLowPass.Ti = 6;
  pidAndLowPass.Ts = Ts;

  pinMode(AirHeaterPin, OUTPUT);
  Serial.begin(9600);

  ThingSpeak.begin(client);  // Initialize ThingSpeak
}

void loop() {
  // put your main code here, to run repeatedly:
  CheckWiFi();
  ConnectWiFi();
  y = ReadTemperature();
  y = pidAndLowPass.LowPassFilter(y);
  u = pidAndLowPass.PiController(y);
  WriteAnalogControlSignal(u);
  if (count == (wait/Ts)){
    count = 0;
    ThingSpeakWrite(y);
  }
  count = count + 1;
  delay(Ts*1000); //Wait 0.1 second for every loop
}

//Write Analog Control Signal
void WriteAnalogControlSignal(float u)
{
  float value = 51 * u;
  analogWrite(AirHeaterPin,value);
}

//Read Temperature
float ReadTemperature()
{
  //Temperature sensor
  float adcValue;
  float voltage;
  float Tout;
  adcValue = analogRead(temp_sensPin);
  voltage = (adcValue*5)/1023;
  Tout = 12.5*voltage - 12.5;
  return Tout;
}

void CheckWiFi()
{
  // check for the presence of the WiFi shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  String fv = WiFi.firmwareVersion();
  if (fv != "1.5.0") {
    Serial.println("Please upgrade the firmware");
  }
}

void ConnectWiFi()
{
  char ssid[] = SECRET_SSID;   // your network SSID (name) 
  char pass[] = SECRET_PASS;   // your network password
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
}

void ThingSpeakWrite(float temperatureValue)
{
  unsigned long myChannelNumber = SECRET_CH_ID;
  const char * myWriteAPIKey = SECRET_WRITE_APIKEY;
  int channelField = 1;
  
  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
  // pieces of information in a channel.  Here, we write to field 1.
  int x = ThingSpeak.writeField(myChannelNumber, channelField, temperatureValue, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  Serial.println(temperatureValue);
}