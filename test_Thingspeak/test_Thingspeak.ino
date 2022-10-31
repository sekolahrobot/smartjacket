/*
 WizFi360 example: Thingspeak

 This sketch Request to thingspeak using a WizFi360 module to
 perform a simple store and search data.
*/

#include <stdlib.h>

#include "WizFi360.h"

// setup according to the device you use
#define WIZFI360_EVB_PICO
#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

MAX30105 particleSensor;
#define SDA_PIN    16        // SDA PIN
#define SCL_PIN    17        // SCL PIN
#define I2C_CLK_FREQ 400000 // Fast I2C 400kHz Clk

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
#if defined(ARDUINO_MEGA_2560)
SoftwareSerial Serial1(6, 7); // RX, TX
#elif defined(WIZFI360_EVB_PICO)
SoftwareSerial Serial2(6, 7); // RX, TX
#endif
#endif

/* Baudrate */
#define SERIAL_BAUDRATE   115200
#if defined(ARDUINO_MEGA_2560)
#define SERIAL1_BAUDRATE  115200
#elif defined(WIZFI360_EVB_PICO)
#define SERIAL2_BAUDRATE  115200
#endif

/* Sensor */
#define DHTTYPE DHT11
#define DHTPIN SDA
#define CDSPIN A0

/* Wi-Fi info */
char ssid[] = "SeRI";       // your network SSID (name)
char pass[] = "sadapsadap";   // your network password

int status = WL_IDLE_STATUS;  // the Wifi radio's status

char server[] = "api.thingspeak.com"; // server address
String apiKey ="8RSVVOI5FPYXLO9W";                    // apki key

// sensor buffer
char temp_buf[10];
char humi_buf[10];
char cds_buf[10];

unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 30000L; // delay between updates, in milliseconds

// Initialize the Ethernet client object
WiFiClient client;
// Initialize temp
int ThermistorPin = 26;
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

#define LED      25
#define ON  HIGH
#define OFF LOW
float compAngleX=0, compAngleY=0, compAngleZ=0; // Calculate the angle using a complementary filter
boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred

int angleChange=0;

void setup() {


  // initialize serial for debugging
  Serial.begin(SERIAL_BAUDRATE);
  // initialize serial for WizFi360 module
#if defined(ARDUINO_MEGA_2560)
  Serial1.begin(SERIAL1_BAUDRATE);
#elif defined(WIZFI360_EVB_PICO)
  Serial2.begin(SERIAL2_BAUDRATE);
#endif
  // initialize WizFi360 module
#if defined(ARDUINO_MEGA_2560)
  WiFi.init(&Serial1);
#elif defined(WIZFI360_EVB_PICO)
  WiFi.init(&Serial2);
#endif

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("You're connected to the network");
  
  printWifiStatus();
  Serial.begin(115200);
  Serial.println("Initializing...");
   Wire.setClock(I2C_CLK_FREQ);
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_CLK_FREQ)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //First transmitting
  sensorRead();
  thingspeakTrans();
}

void loop() {
  // if there's incoming data from the net connection send it out the serial port
  // this is for debugging purposes only   
  while (client.available()) {
    char c = client.read();
    Serial.print("recv data: ");
    Serial.write(c);
    Serial.println();
  }
  
  // if 30 seconds have passed since your last connection,
  // then connect again and send data
  if (millis() - lastConnectionTime > postingInterval) {
    sensorRead();
    thingspeakTrans();
  }
}

// Read sendsor value
void sensorRead() {
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = (T - 273.15);
  Tf = (Tc * 9.0)/ 5.0 + 32.0; 
  
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
 

  if (irValue < 50000)
    Serial.print(" No finger/wrist ?");

  Serial.println();
}

//Transmitting sensor value to thingspeak
void thingspeakTrans() {
  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection
  if (client.connect(server, 80)) {
    Serial.println("Connecting...");
    
    // send the Get request
    client.print(F("GET /update?api_key="));
    client.print(apiKey);
    client.print(F("&field1="));
    client.print(beatAvg);
    client.print(F("&field2="));
    client.print(Tc);
//    client.print(F("&field3="));
//    client.print(cds_buf);
    client.println();
    // note the time that the connection was made
    lastConnectionTime = millis();
  }
  else {
    // if you couldn't make a connection
    Serial.println("Connection failed");
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void mpu_read(){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 }
