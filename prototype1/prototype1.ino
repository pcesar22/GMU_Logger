
// Number of variables & functions
#define NUMBER_VARIABLES 2
#define NUMBER_FUNCTIONS 1

// Include required libraries
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include <CC3000_MDNS.h>
#include <aREST.h>
#include <avr/wdt.h>

// Define CC3000 chip pins
#define ADAFRUIT_CC3000_IRQ   3
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10

#define SHARP_ADDRESS   0x01
#define SENSOR_COUNT    1
#define BUFFER_SIZE     1000

// WiFi network (change with your settings !)
#define WLAN_SSID       "PCWireless"
#define WLAN_PASS       "ad23=cent96"
#define WLAN_SECURITY   WLAN_SEC_WPA2

// Create CC3000
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, 
ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT, SPI_CLOCK_DIV2);

// Create aREST instance
aREST rest = aREST();

// The port to listen for incoming TCP connections 
#define LISTEN_PORT           80

// Server instance
Adafruit_CC3000_Server restServer(LISTEN_PORT);


// Infrared sensor pin
#define SENSPIN   A0
#define LEDPIN    2
// DNS responder instance
MDNSResponder mdns;

int temperature,humidity;


///////////////////////
// Memory Allocation //
///////////////////////

// Storage
float buffer[BUFFER_SIZE];
int bufferIndex;

//////////////////////////
// Function prototypes  //
//////////////////////////

float sharpProcessor_f(int);
void updateSensorReadings(void);
void bufferDump(void);

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



// List components
// struct analogSensorNode *root;
// struct analogSensorNode *conductor;

char incomingByte = 0;

void setup() {
  humidity = 0;

  // Start serial
  Serial3.begin(9600);
  Serial.begin(115200);

  // Function to be exposed
  rest.function("led",ledControl);
  // Variables
  rest.variable("temperature", &temperature);
  rest.variable("humidity", &humidity);

  // Set name
  rest.set_id("1");
  rest.set_name("weather_station");

  // Start bufferIndex
  bufferIndex = 0;

  // Initialize the address, hw and process functions
  sensorList[0].address = addressList[0];
  sensorList[0].pin = pinList[0];
  sensorList[0].func_pointer = funcList[0];



  ccInit();

 // Start server
  restServer.begin();
  Serial.println(F("Listening for connections..."));
  
}

void loop() {

  for(int i = 0 ; i < SENSOR_COUNT ; i++){
    updateSensorReadings();
    //Serial.println(sensorList[i].processedReading);
    temperature = sensorList[i].rawReading; // will become a vector of values
  }
  humidity = 0;
  
  // Handle any multicast DNS requests
  mdns.update();

  // Handle REST calls
  Adafruit_CC3000_ClientRef client = restServer.available();
  rest.handle(client);

  // Do internet stuff here
  
  // while(Serial.available() > 0){
  //   incomingByte = Serial.read();
  //   Serial3.write(incomingByte);
  //   Serial.print(incomingByte);
  // }
  // while(Serial3.available() > 0){
  //   incomingByte = Serial3.read();
  //   Serial.print(incomingByte);
  // }
 }


// Simple ultra-random function to calculate actual values
float sharpProcessor_f(int rawValue){
  float distance = 1.0f*rawValue/1024;
  distance = 100 - 90/0.57*distance;
  return distance;
}

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

void bufferDump(void){
  for(int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }

}


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
  
   // Start multicast DNS responder
  if (!mdns.begin("arduino", cc3000)) {
    while(1); 
  }

  Serial.println("OK!... Starting server");
  
  // Start server
  restServer.begin();

  displayConnectionDetails(); 
}

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

int ledControl(String command) {
  
  // Get state from command
  int state = command.toInt();
  
  digitalWrite(2,state);
  return 1;
}