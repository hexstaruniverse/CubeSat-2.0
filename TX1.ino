#include <VirtualWire.h>                            //Library for the Radios
#include "DHT.h"                                    //Library for the DHT11 Sensor
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <MechaQMC5883.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//-----------------------------------------------------------------------------------------------------
//                                               DEFINES
//-----------------------------------------------------------------------------------------------------

#define DHTPIN  2                                //DHT Output on D4
#define DHTTYPE DHT11                               //DHT Type (11,21,22)

//-----------------------------------------------------------------------------------------------------
//                                              VARIABLES
//-----------------------------------------------------------------------------------------------------
const int led_pin = 13;                             //LED on D13
const int transmit_pin = 12; 
static const int RXPin = 3, TXPin = 4;// Here we make pin 4 as RX of arduino & pin 3 as TX of arduino 
static const uint32_t GPSBaud = 9600;                       //Radio input on D12
//-----------------------------------------------------------------------------------------------------
//                                           DATA STRUCTURES
//-----------------------------------------------------------------------------------------------------

// If you are not familiar with Data Structures visit this link http://www.cplusplus.com/doc/tutorial/structures/

struct package                                      //Struct Type Name
{
  float temperature ;                               //Struct Member
  float humidity ; 
  float pressure;
  float altitude;                                 //Struct Member
  float accex;
  float accey;
  float accez;
  float gyrx;
  float gyry;
  float gyrz;
  int x, y, z;
  int azimuth;
  int heading;
  double lat;
  double lon;
};


typedef struct package Package;                     //Define name of
Package data;                                       //Object name

//-----------------------------------------------------------------------------------------------------
//                                            LIBRARY CALLS
//-----------------------------------------------------------------------------------------------------
DHT dht(DHTPIN, DHTTYPE);                           //Create instance of DHT called dht
Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
MechaQMC5883 qmc;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

//-----------------------------------------------------------------------------------------------------
//                                                SETUP
//-----------------------------------------------------------------------------------------------------
void setup()
{
    
    vw_set_tx_pin(transmit_pin);                    //initialize radio on D12
    vw_set_ptt_inverted(true);                      // Required for DR3100
    vw_setup(10000);                                  // Bits per sec
    pinMode(led_pin, OUTPUT);                       //set LED for output
     // set accelerometer range to +-8G
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.begin(9600);
    ss.begin(GPSBaud);
}

//-----------------------------------------------------------------------------------------------------
//                                              MAIN LOOP
//-----------------------------------------------------------------------------------------------------

void loop()
{
  float altitude;    
  float pressure;
  digitalWrite(led_pin, HIGH);                      // Flash a light to show transmitting
  readSensor();                                     //Call readSensor function
  vw_send((uint8_t *)&data, sizeof(data));          //send data (struct array)
  vw_wait_tx();                                     // Wait until the whole message is gone
  digitalWrite(led_pin, LOW);                       //LED off
  delay(1);                                      //wait 2 seconds
  altitude = bmp.readAltitude();
  pressure = bmp.readPressure();
  // For GPS Comment off this code.
  //while (ss.available() > 0)
   // if (gps.encode(ss.read()))
     // displayInfo();

 // if (millis() > 5000 && gps.charsProcessed() < 10)
  //{
   // Serial.println(F("No GPS detected: check wiring."));
    //while(true);
  //}

}

//void displayInfo()
//{
  //Serial.print(F("Location: ")); 
  //if (gps.location.isValid())
  //{
    //data.lat = gps.location.lat();
    //data.lon = gps.location.lng();
    
  //}
  //else
  //{
    //Serial.print(F("INVALID"));
  //}
  //Serial.println();
//}
//-----------------------------------------------------------------------------------------------------
//                                              FUNCTIONS
//-----------------------------------------------------------------------------------------------------
void readSensor()                                   //readSensor function
{
  qmc.init();
  int x, y, z;
  int azimuth;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  qmc.read(&x, &y, &z, &azimuth);
  int heading=atan2(x, y)/0.0174532925;
 mpu.begin();
 bmp.begin();
 dht.begin();                                       //initialize dht
 ss.begin(GPSBaud);

 
 delay(1);                                       //wait 1 second
 data.humidity = dht.readHumidity();                //get humidity & store in Struct variable data.humidity
 data.temperature = dht.readTemperature();          //get temperature & store in Struct variable data.temperature
 data.altitude = bmp.readAltitude();
 data.pressure = bmp.readPressure();
 data.gyrx = a.gyro.x;
 data.gyry = a.gyro.y;
 data.gyrz = a.gyro.z;
 data.accex = a.acceleration.x;
 data.accey = a.acceleration.y;
 data.accez = a.acceleration.z;

 data.x = x;
 data.y = y;
 data.z = z;
 data.azimuth = azimuth;
 data.heading = heading;
 

}
