#include <RG11.h>
#include <avr/pgmspace.h>
#include <Time.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>

/**********************************************
/*
/*      DATALOGGER DEFINITIONS
/*                       
/**********************************************/

// Timer.h specific defines
#define TIME_HEADER   "T"
#define TIME_REQUEST  7            

#define BUFFER_SIZE               400

// SD card CS pin ... 53 on the atmega2560
// See https://learn.adafruit.com/downloads/pdf/adafruit-micro-sd-breakout-board-card-tutorial.pdf
#define CS_PIN                    53

#define BUCKET_VOLUME 0.01f

//////////////////////
// type definitions //
//////////////////////


// Type for storing the rain level readings
typedef struct rainReading_t{

  float value;
  time_t timestamp;


} rainReading_t;


///////////////////
// Declare stuff //
///////////////////

// Declare rain sensor
RainSensor rainSensor;

///////////////////////
// Memory Allocation //
///////////////////////

rainReading_t rainBuffer[BUFFER_SIZE];
int rainBufferIndex,lastBufferIndex;

File logfile;




// Other initializations
unsigned long oldTime = 0, newTime = 0, 
oldTime2 = 0, newTime2 = 0,
oldTime3 = 0, newTime3 = 0,
oldTime4 = 0;


/**********************************************
/*
/*      MAIN ARDUINO CODE
/*                       
/**********************************************/


void setup() {
  // set time 
  setTime(1,0,0,22,7,2015);

  // IO 
  pinMode(13,OUTPUT);
  pinMode(2, INPUT);
  pinMode(CS_PIN, OUTPUT);


  // Can only debug this on USB connected Arduino
  if(!SD.begin(CS_PIN)){
    Serial.println("Test failed");
  }

  // Interrupt for rain sensor. The first parameter, "0" means
  // pin 2 on the atmega2560. Everytime a falling edge occurs,
  // the program will halt and the "rainDetected" function will 
  // run
  attachInterrupt(0, rainDetected, FALLING);

  // begin rainsensor
  rainSensor.init();

  // Start serial
  Serial3.begin(9600);
  Serial.begin(9600);

  
  // Start bufferIndex
  rainBufferIndex = 0;
  lastBufferIndex = 0;

}

void loop() {

  // Main time values ... < CHANGE HERE >
  unsigned int timeForRainToStop = 15; // in seconds 
  unsigned int timeToDumpReadingsUART = 15; // in seconds
  unsigned int timeToAddReadingToBuffer = 3; // in seconds

  // Pseudo timer interrupt 1 ... Signal that rain has stopped
  newTime = millis();
  if (newTime >= oldTime + timeForRainToStop*1000)
  {
    oldTime = millis();
    rainHasStopped();
  }else if(newTime < oldTime){ // means a reset occurred. Happens after 50 days
    oldTime = 0;
  }

  // Pseudo timer interrupt 2 ... dump the readings in UART
  newTime2 = millis();
  if(newTime2 >= oldTime2 + timeToDumpReadingsUART*1000)
  {
    oldTime2 = millis();
    rainBufferDump();
  }else if(newTime2 < oldTime2){
    oldTime2 = 0;
  }

 // Pseudo timer interrupt 3 ... add reading to buffer
  newTime3 = millis();
  if(newTime3 >= oldTime3 + timeToAddReadingToBuffer*1000)
  {
    oldTime3 = millis();
    addToRainBuffer();
  }else if(newTime3 < oldTime3){
    oldTime3 = 0;
  }

  if (Serial.available()) { 
    processSyncMessage();
  }

  delay(100);

  // Confirm if date is set
  if (timeStatus() == timeSet) {
    digitalWrite(13, HIGH); // LED on if synced
  } else {
    digitalWrite(13, LOW);  // LED off if needs refresh
  }


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


//////////////////////////////////////////////////////////
// rain buffer stuff -> where all information is stored //
//////////////////////////////////////////////////////////

// Dump information out the UART and store it to program memory
void rainBufferDump(void){

  Serial.println("Enter rainBufferDump() (every 20 secs)");
  time_t currentTime = now();

  char buf[20];

  

  // Create the filename for the SD card entry
  char filename[12];


  itoa(day(currentTime),filename,10);
  strcat(filename,"-");
  strcat(filename,itoa(month(currentTime),buf,10));
  strcat(filename,".txt");

  logfile = SD.open(filename, FILE_WRITE);

  char year_i[6];
  char month_i[4];
  char day_i[4];
  char hour_i[4];
  char min_i[4];
  char second_i[4];

  char period_i[5];


  for(int i = lastBufferIndex; i < rainBufferIndex; i++){
    Serial.print("Height: ");
    Serial.print(rainBuffer[i].value); 
    Serial.print(" inches. --- ");

    time_t stamp = rainBuffer[i].timestamp;
    char dateEntry[] = "Date: ";
    itoa(year(stamp),year_i,10);
    itoa(month(stamp),month_i,10);
    itoa(day(stamp),day_i,10);
    itoa(hour(stamp),hour_i,10);
    itoa(minute(stamp),min_i,10);
    itoa(second(stamp),second_i,10);
    // strcpy(period_i,"AM");
    // if(isPM(stamp)) strcpy(period_i,"PM");


    strcat(dateEntry,hour_i);
    strcat(dateEntry,":");
    strcat(dateEntry,min_i);
    strcat(dateEntry,":");
    strcat(dateEntry,second_i);
    //strcat(dateEntry,period_i);

    strcat(dateEntry," ");
    strcat(dateEntry,month_i);
    strcat(dateEntry,"/");
    strcat(dateEntry,day_i);
    strcat(dateEntry,"/");
    strcat(dateEntry,year_i);

    Serial.println(dateEntry);
    // Print to SD card
    if(logfile){
      logfile.print("Height: ");
      logfile.print(rainBuffer[i].value);
      logfile.print(" inches. --- ");
      logfile.println(dateEntry);
      logfile.close();
    }
  }
  delay(100);

  lastBufferIndex = rainBufferIndex;
  Serial.println("exit  --");
  

}

void addToRainBuffer(){
  Serial.println(F("Enter addToRainBuffer() (should happen every 3 seconds)"));
  rainBuffer[rainBufferIndex].value = rainSensor.getRainHeightInInches();
  rainBuffer[rainBufferIndex].timestamp = now();
  rainBufferIndex++;
  if(rainBufferIndex == BUFFER_SIZE){
    Serial.println(F("rainBufferIndex reset occurred"));
    rainBufferIndex = 0;
  }
  Serial.print("rainBufferIndex: ");
  Serial.println(rainBufferIndex);
}

/**********************************************
/*
/*      Helper functions
/*                       
/**********************************************/



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
    if(millis() - oldTime >= 300){ //debounce
      oldTime = millis();
      rainSensor.addHeightInInches(BUCKET_VOLUME);
      Serial.print(F("New volume:"));
      Serial.println(rainSensor.getRainHeightInInches());
    }

  }else{
    Serial.println(F("Rain Detected! Start counting"));
    rainSensor.setRainStarted();

  }
  
}

void rainHasStopped(){
  if(rainSensor.isRaining()){
    rainSensor.setRainStopped();
    Serial.println(F("Oh no! Rain has stopped!"));
  }
}
