/*
 * Arduino Wireless Weather Station
 * Receiver Code
 *
 * 
 * by Hex-Star Universe
 *
 */




//-----------------------------------------------------------------------------------------------------
//                                            LIBRARIES
//-----------------------------------------------------------------------------------------------------

#include <VirtualWire.h>


//-----------------------------------------------------------------------------------------------------
//                                            VARIABLES
//-----------------------------------------------------------------------------------------------------
const int led=13;
const int receive_pin = 12;
char temperatureChar[10];
char humidityChar[10];
char AltitudeChar[10];
char PressureChar[10];
char accelerationChar[10];

//-----------------------------------------------------------------------------------------------------
//                                          DATA STRUCTURE
//-----------------------------------------------------------------------------------------------------
struct package                                      //Struct Type Name
{
  float temperature = 0.0;                          //Struct Member                        
  float humidity = 0.0;                             //Struct Member
  float altitude; 
  float pressure;
  float accex;
  float accey;
  float accez;
  float gyrx;
  float gyry;
  float gyrz;
  int x, y, z;
  int azimuth;
  int heading;
};


typedef struct package Package;                     //Define name of
Package data;                                       //Object name

//-----------------------------------------------------------------------------------------------------
//                                              SETUP
//-----------------------------------------------------------------------------------------------------
void setup()
{
    
Serial.begin(9600);                                 //start serial comms for output
    
    pinMode(led,OUTPUT);                            //LED set for output
    vw_set_rx_pin(receive_pin);                     //initialize receiver on pin D12
    vw_setup(10000);                                  // Bits per sec
    vw_rx_start();                                  // Start the receiver PLL running
}


//-----------------------------------------------------------------------------------------------------
//                                            MAIN LOOP
//-----------------------------------------------------------------------------------------------------
void loop()
{
    uint8_t buf[sizeof(data)];                      //unsigned int 8 bit called buf with size specified in Struct member data
    uint8_t buflen = sizeof(data);                  //unsigned int 8 bit called buflen with size specified in Struct member data

if (vw_have_message())                              //Check for activity 
  {
    digitalWrite(led,HIGH);                         //flash LED
    vw_get_message(buf, &buflen);                   //get the data, store in variable buf with length of bufflen
    memcpy(&data,&buf,buflen);                      //memcpy ( void * destination, const void * source, size_t num );

    Serial.print("T    ");                      //print header to serial port
    Serial.print(data.temperature);        //print temperature in F to serial port
    Serial.println("C");
                                //print units to serial port with CR

    Serial.print("P   ");                      //print header to serial port
    Serial.print(data.altitude);        //print temperature in F to serial port
    Serial.println("pa"); 
    

    Serial.print("A   ");                      //print header to serial port
    Serial.print(data.pressure);        //print temperature in F to serial port
    Serial.println("m");
     
    
    Serial.print("Gx  ");                      //print header to serial port
    Serial.print(data.gyrx);        //print temperature in F to serial port
    Serial.println("");
     

    Serial.print("Gy  ");                      //print header to serial port
    Serial.print(data.gyry);        //print temperature in F to serial port
    Serial.println("");
    

    Serial.print("Gz   ");                      //print header to serial port
    Serial.print(data.gyrz);        //print temperature in F to serial port
    Serial.println("");
     

    Serial.print("H ");                      //print header to serial port
    Serial.print(data.humidity);                    //print humidity to serial port
    Serial.println("%");                            //print units to serial port with CR


    Serial.print("X ");                      //print header to serial port
    Serial.print(data.x);                    //print humidity to serial port
    Serial.println("uT"); 

    Serial.print("Y ");                      //print header to serial port
    Serial.print(data.y);                    //print humidity to serial port
    Serial.println("uT");

    Serial.print("Z");                      //print header to serial port
    Serial.print(data.z);                    //print humidity to serial port
    Serial.println("uT");

    Serial.print("Azimuth ");                      //print header to serial port
    Serial.print(data.azimuth);                    //print humidity to serial port
    Serial.println("");

    Serial.print("Heading ");                      //print header to serial port
    Serial.print(data.heading);                    //print humidity to serial port
    Serial.println("");

    Serial.print("Ax ");                      //print header to serial port
    Serial.print(data.accex);                    //print humidity to serial port
    Serial.println("");

    Serial.print("Ay ");                      //print header to serial port
    Serial.print(data.accey);                    //print humidity to serial port
    Serial.println("");

    Serial.print("Az ");                      //print header to serial port
    Serial.print(data.accez);                    //print humidity to serial port
    Serial.println("");

    
    Serial.println("___________");         
    
    delay(1);                                     //wait 100 miliseconds
    digitalWrite(led,LOW);                          //led off
  }
}


