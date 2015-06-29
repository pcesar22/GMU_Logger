#include <Phant.h>

char pub_key = EJE9a168KNFy1O5WdVyj;
char priv_key = dqMz6G4NVbuYeXyzDgYA;



#define SHARP_ADDRESS   0x01
#define SENSOR_COUNT    1
#define BUFFER_SIZE     1000
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
struct analogSensorNode *root;
struct analogSensorNode *conductor;

char incomingByte = 0;

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(9600);
  Serial.begin(9600);

  // Start bufferIndex
  bufferIndex = 0;
  // Initialize the address, hw and process functions
  sensorList[0].address = addressList[0];
  sensorList[0].pin = pinList[0];
  sensorList[0].func_pointer = funcList[0];

  //eremy.george@digi.com
  
}

void loop() {

  for(int i = 0 ; i < SENSOR_COUNT ; i++){
    updateSensorReadings();
    Serial.println(sensorList[i].processedReading);
  }

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

    phant.add("")
  }

}
