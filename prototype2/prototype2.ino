
// Include required libraries
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include "string.h"
#include "ccspi.h"
#include "utility/debug.h"
#include <Phant.h>


/**********************************************
    CC3000 DEFINITIONS                       
**********************************************/

// Define CC3000 chip pins
#define ADAFRUIT_CC3000_IRQ            3     
#define ADAFRUIT_CC3000_VBAT           5
#define ADAFRUIT_CC3000_CS             10

#define SHARP_ADDRESS                  0x01
#define SENSOR_COUNT                   1
#define BUFFER_SIZE                    1000 

// WiFi network (change with your settings !)
#define WLAN_SSID       "PCWireless"
#define WLAN_PASS       "ad23=cent96"
#define WLAN_SECURITY   WLAN_SEC_WPA2
#define WEBSITE         "data.sparkfun.com"

// Keys for data.sparkfun access
const char pub_key[] = "EJE9a168KNFy1O5WdVyj";
const char priv_key[] = "dqMz6G4NVbuYeXyzDgYA";

// Create CC3000
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, 
ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIV2);

// ip address
uint32_t ip;

// The port to listen for incoming TCP connections 
#define LISTEN_PORT           80

// Client instance
Adafruit_CC3000_Client myClient;

/**********************************************
    DATALOGGER DEFINITIONS                  
**********************************************/

// Infrared sensor pin
#define SENSPIN   A0
#define LEDPIN    2

const byte NUM_FIELDS = 1;
const String fieldNames[NUM_FIELDS] = {"distance"};
String fieldData[NUM_FIELDS];

///////////////////////
// Memory Allocation //
///////////////////////

// Storage
float buffer[BUFFER_SIZE];
int bufferIndex;

//////////////////////
// type definitions //
//////////////////////

// Function pointers for reading conversion
typedef float (*convFunc_ptr_t)(int);

// hw pin type
typedef int pin_t;

// Analog sensor structure
typedef struct analogSensor_t {


    uint8_t address;
    //uint8_t samplePeriod;

    // Readings
    int rawReading;
    float processedReading;

    // Reading function processor
    convFunc_ptr_t func_pointer;

    // HW pin being used
    int pin;

    

} analogSensor_t;


// Node for use of lists.
typedef struct analogSensorNode{
  analogSensor_t aSensor;
  struct analogSensorNode *next;
};


//////////////////////////
// Sensor declaration   //
//////////////////////////


// Declare analog sensors
analogSensor_t sharpSensor;

// Place the analog sensors in a list, as well as their
// hardware/software instantiation information

// Analog sensor list
analogSensor_t sensorList[] = {sharpSensor};

// Analog processor functions (convert raw 0-1023 into SI units)
convFunc_ptr_t funcList[] = {sharpProcessor_f};

// Sensor Addresses for later use
uint8_t addressList[] = {SHARP_ADDRESS};

// Hardware pins used by the sensors
pin_t pinList[] = {A0};


/**********************************************
    MAIN ARDUINO CODE                       
**********************************************/


void setup() {
  pinMode(2,OUTPUT);

  // Start serial
  Serial3.begin(9600);
  Serial.begin(115200);


  // Start bufferIndex
  bufferIndex = 0;

  // Initialize the address, hw and process functions
  sensorList[0].address = addressList[0];
  sensorList[0].pin = pinList[0];
  sensorList[0].func_pointer = funcList[0];

  // Connect to the Internet
  ccInit();

  
  // Try looking up the website's IP address
  ip = 0;
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  Serial.print("The serial ip is: ");
  Serial.println(ip);
  
  Serial.println(F("Server online! Listening for connections..."));

  // Updates the website 5 times
  int counter = 0;
  while(counter < 5){
    digitalWrite(2,HIGH);
    for(int i = 0 ; i < SENSOR_COUNT ; i++){
      updateSensorReadings();
      String dataString = String(int(sensorList[i].processedReading)) + "." 
        + String(getDecimal(sensorList[i].processedReading));
      
      fieldData[i] = dataString;
    }
    Serial.println("\nPosting data!\n");
    postData();
    counter++;
    digitalWrite(2,LOW);
    delay(2000);
  }
  Serial.println("Disconnecting");
  cc3000.disconnect();
  Serial.println("... done!");
  digitalWrite(2,LOW); 
  
}

void loop() {
  
  delay(1000);
}


/**********************************************
    HELPER FUNCTIONS                       
**********************************************/


////////////////////////////////////////////////
// Conversion functions (each sensor has own) //
////////////////////////////////////////////////

// Infrared Sharp Sensor
float sharpProcessor_f(int rawValue){
  float distance = 1.0f*rawValue/1024;
  distance = 100 - 90/0.57*distance;
  return distance;
}


// Updates all the sensor readings
void updateSensorReadings(){
  for(int i = 0; i < SENSOR_COUNT ; i++){
    // get reading from ping
    sensorList[i].rawReading = analogRead(sensorList[i].pin);
    
    // process reading
    sensorList[i].processedReading  = 
      sensorList[i].func_pointer(sensorList[i].rawReading);
    
    // Add to buffer, for now it's global
    buffer[bufferIndex] = sensorList[i].processedReading;

    if(bufferIndex > BUFFER_SIZE) bufferIndex = 0;
    else bufferIndex++;
    delay(5);

  }
}

// Prints out all the data inside the Buffer
void bufferDump(void){
  for(int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }
}



// initialize connection
void ccInit(){
  // Initialise the CC3000 module
  Serial.println("Initalizing module");
  if (!cc3000.begin())
  {
    while(1);
  }
  Serial.println("Ok! Connecting to wifi network");
  // Connect to  WiFi network
  cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);
    
  Serial.println("OK! Getting DHCP and initializing DNS responder");
  // Check DHCP
  while (!cc3000.checkDHCP())
  {
    delay(100);
  }  
  
 
  displayConnectionDetails(); 
}

// Displays all the ongoing connections
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

// posts the data to the data.sparkfun.com/gmu_logger server
void postData(){
  myClient = cc3000.connectTCP(ip,LISTEN_PORT);
  if(myClient.connected()){
    Serial.println("Connected to Spark fun data server!");

    myClient.print("POST /input/");
    myClient.print(pub_key);
    myClient.print("?private_key=");
    myClient.print(priv_key);
    for(int i = 0; i < NUM_FIELDS; i++){
      myClient.print("&");
      myClient.print(fieldNames[i]);
      myClient.print("=");
      myClient.print(fieldData[i]);
    }
    myClient.println(" HTTP/1.1");
    myClient.print("Host: ");
    myClient.println(WEBSITE);
    myClient.println("Connection: close");
    myClient.println();

    while (myClient.connected())
    {
      if ( myClient.available() )
      {
        char c = myClient.read();
        Serial.print(c);
      }      
    }
    Serial.println();
  }else{
    Serial.println(F("Connection failed"));
    return;
  }

  Serial.println("=====================================");
  
}

// Gets the decimal part of a floating point value, two digits
int getDecimal(float value){
  return (value-1.0f*(int)value)*100;
}