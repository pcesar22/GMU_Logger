#include <RG11.h>
#include <avr/pgmspace.h>
#include <Time.h>
#include <RG11.h> 
#include <TimerOne.h>
/**********************************************
/*
/*      DATALOGGER DEFINITIONS
/*                       
/**********************************************/


#define TIME_HEADER   "T"
#define TIME_REQUEST  7            

// Infrared sensor pin
#define SENSPIN                   A0
#define LEDPIN                    2
#define BUFFER_SIZE               1000
#define SENSOR_COUNT              1
#define SHARP_ADDRESS             0x00

#define BUCKET_VOLUME 0.01f

const byte NUM_FIELDS = 2 ;
const String fieldNames[NUM_FIELDS] = {"distance","rain"};
String fieldData[NUM_FIELDS];

///////////////////////
// Memory Allocation //
///////////////////////

// Storage
float buffer[BUFFER_SIZE];
float rainBuffer[BUFFER_SIZE];
time_t time_stamps[BUFFER_SIZE]; 

int bufferIndex;
int rainBufferIndex;

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




//////////////////////////
// Sensor declaration   //
//////////////////////////

// Declare rain sensor
RainSensor rainSensor;

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

// Other initializations
unsigned long oldTime = 0, newTime = 0, oldTime2 = 0, newTime2 = 0;

/**********************************************
/*
/*      MAIN ARDUINO CODE
/*                       
/**********************************************/


void setup() {

  // IO 
  pinMode(13,OUTPUT);
  pinMode(2, INPUT);

  // Interrupt for rain sensor
  attachInterrupt(0, rainDetected, FALLING);

  // begin rainsensor
  rainSensor.init();

  // Start serial
  Serial3.begin(9600);
  Serial.begin(9600);

  
  // Start bufferIndex
  bufferIndex = 0;
  rainBufferIndex = 0;

  // Initialize the address, hw and process functions
  sensorList[0].address = addressList[0];
  sensorList[0].pin = pinList[0];
  sensorList[0].func_pointer = funcList[0];

}

void loop() {
  // Pseudo timer interrupt 1
  newTime = millis();
  if (newTime >= oldTime + 15000) // 15 seconds
  {
    oldTime = millis();
    rainHasStopped();
  }else if(newTime < oldTime){ // means a reset occurred. Happens after 50 days
    oldTime = 0;
  }

  // Pseudo timer interrupt 2
  newTime2 = millis();
  if(newTime2 >= oldTime2 + 10000) // 10 seconds
  {
    oldTime2 = millis();
    rainBufferDump();
  }else if(newTime2 < oldTime2){
    oldTime2 = 0;
  }



  if (Serial.available()) { 

    processSyncMessage();
  }
  // Confirm if date is set
  if (timeStatus() == timeSet) {
    digitalWrite(13, HIGH); // LED on if synced
  } else {
    digitalWrite(13, LOW);  // LED off if needs refresh
  }

  addToRainBuffer();
  delay(1000); // readings every 1000 ms

  //updateAnalogSensorReadings();
  // digitalWrite(2,HIGH);
  // Serial3.println("Hello World!");
  // Serial.println(sensorList[0].processedReading);
  
  // delay(400);
  // digitalWrite(2,LOW);
  // delay(400);
}

/**********************************************
/*
/*      Sensor functions
/*                       
/**********************************************/


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
void updateAnalogSensorReadings(){
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

/**********************************************
/*
/*      Helper functions
/*                       
/**********************************************/

// Prints out all the data inside the Buffer
void bufferDump(void){
  for(int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }
}

void rainBufferDump(void){
  for(int i = 0; i < rainBufferIndex; i++){
    Serial.print(rainBuffer[i]); 
    Serial.print("   ----   ");
    Serial.println(time_stamps[i]);
  }
}

void addToRainBuffer(){
  rainBuffer[rainBufferIndex] = rainSensor.getRainHeightInInches();
  time_stamps[rainBufferIndex] = now();
}

// Gets the decimal part of a floating point value, two digits
int getDecimal(float value){
  return (value-1.0f*(int)value)*100;
}

// Helper function to communicate with Serial port 3
void talkWithSerial3(void){
  char incomingByte = 0;
  while(Serial.available() > 0){
    incomingByte = Serial.read();
    Serial3.write(incomingByte);
    Serial.print(incomingByte);
  }
  while(Serial3.available() > 0){
    incomingByte = Serial3.read();
    Serial.print(incomingByte);
  }
}

/**********************************************
/*
/*      Timer functions
/*                       
/**********************************************/

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void processSyncMessage() {
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
       setTime(pctime); // Sync Arduino clock to the time received on the serial port
     }
  }
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; // the time will be sent later in response to serial mesg
}

/**********************************************
/*
/*      Interrupt Routines
/*                       
/**********************************************/

void rainDetected(){

  if(rainSensor.isRaining()){
    if(millis() - oldTime >= 100){
      oldTime = millis();
      rainSensor.addHeightInInches(BUCKET_VOLUME);
    }

  }else{
    Serial.println("Rain Detected! Start counting");
    rainSensor.setRainStarted();

  }
  
}

void rainHasStopped(){
  if(rainSensor.isRaining()){
    rainSensor.setRainStopped();
    Serial.println("Oh no! Rain has stopped!");
  }
}
