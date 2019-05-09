//declare input and output pins
double throttleValue = 0;
int motor = 6;

//declare global constants and variables
int minStart = 0;
int previousMotorWrite = minStart;
double lowerBound = 0; //165
double higherBound = 1023; //880
const int maxdelay = 1000;

void setup() {
  //set pin types
  pinMode(A0, INPUT);
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  lowerBound = analogRead(A0);
}

//reads and updates throttle signal at start of every tick
void readThrottle() {
  double throttleRead = analogRead(A0);
  if (throttleRead < lowerBound) {
    lowerBound = throttleRead; //set lower bound of throttle signal
  }
  throttleValue = map(throttleRead, lowerBound, higherBound, 0, 255);
  if (throttleValue < 10) {
    throttleValue = 0;
  } else {
    throttleValue = map(throttleValue, 0, 255, minStart, 255);
  }
}

//ramping function called when throttle signal > last written motor signal
//print statements left for debugging purposes
void eStart() {
  Serial.println("ramping");
  for (int i = previousMotorWrite; i <= throttleValue; i++) {
    analogWrite(motor, i);
    Serial.println(i);
    previousMotorWrite = i;
    delay(map(i, minStart, 255, 30, 1));
    readThrottle();
  }
}

//main loop function
void loop() {
  readThrottle();
  if (throttleValue > previousMotorWrite && previousMotorWrite < 200) {
    eStart();
  } else {
    analogWrite(motor, throttleValue);
    Serial.println(throttleValue);
    previousMotorWrite = throttleValue;
  }
  
}
