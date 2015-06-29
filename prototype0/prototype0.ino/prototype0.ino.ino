#define testPin 2

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(9600);
  Serial.begin(9600);
  pinMode(testPin, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial3.print("Hello world!");
  delay(1000);
  
}
