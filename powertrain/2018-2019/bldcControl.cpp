/*---- PINS ----*/
int hallGreen = A0;
int hallGray = A2;
int hallWhite = A5;
int throttleIn = A4;
int aHigh = 10;
int aLow = 6;
int bHigh = 4;
int bLow = 5;
int cHigh = 9;
int cLow = 3;
/*---------------*/

int debugging = 0;
int fullThrottle = 0;

/*
 * Phases:
 * A: Yellow
 * B: Black
 * C: Red
 * 
 * Halls:
 * Green to Gray
 * Gray to Green
 */

/*--STEP TRACKING--*/
int currentStep = 0;
int previousStep = 0;
int previous = 0;
int actualStep = -1;
/*-----------------*/

/*--MISCELLANEOUS--*/
const int hallValuesStored = 5;
/*-----------------*/

/*------RAMP VARIABLES------*/
unsigned long timer = 0;
bool timerStart = false;
int k = 0; 
int motorWrite = 0;
int throttleRead = 0;
int lowerBound = 240;
int upperBound = 760;
/*--------------------------*/

class MotorPhase {

  
  public:
    MotorPhase(int HI, int LO) {
      m_HI = HI;
      m_LO = LO;
    }

    void writePWM(int high) {
      switch (high) {
        case 1:
          analogWrite(m_HI, 255 - motorWrite);
          //digitalWrite(m_HI, LOW);
          analogWrite(m_LO, 0);
          break;
        case -1:
          analogWrite(m_HI, 255);
          analogWrite(m_LO, motorWrite);
          //digitalWrite(m_LO, HIGH);
          break;
        case 0:
          analogWrite(m_HI, 255);
          analogWrite(m_LO, 0);
          break;
      }
    }

  private:
    int m_HI;
    int m_LO;
};

class HallSensor {
  public:

    HallSensor(int inputPin) {
      m_valuesStored = 0;
      m_inputPin = inputPin;
      m_currentState = -1;
    }

    int currentState() const {
      return m_currentState;
    }

    void readNewValue() {
      if (m_valuesStored >= hallValuesStored) {
        for (int i = hallValuesStored - 2; i >= 0; i--) {
          m_previousReadings[i+1] = m_previousReadings[i];
        }
        m_previousReadings[0] = analogRead(m_inputPin);
      }
      else {
        m_previousReadings[m_valuesStored] = analogRead(m_inputPin);
      }

      m_valuesStored += 1;
    }

    bool determineState() { //returns true on state change
      while (m_valuesStored < hallValuesStored) {
        this->readNewValue();
      }

      this->readNewValue();

      bool hasZero = false;
      int nonZeroCount = 0;
      int zeroCount = 0;
      for (int i = 0; i < hallValuesStored; i++) {
        //Serial.println(m_previousReadings[i]);
        if (m_previousReadings[i] > 1000) {
          nonZeroCount += 1;
        } else if (m_previousReadings[i] <= 1000) {
          hasZero = true;
          zeroCount += 1; //added
          //break; //commented
        }
      }

      int previousState = m_currentState;

      /* alternate implementation
      if (nonZeroCount >= hallValuesStored/2) {
        m_currentState = 1;
      } else {
        m_currentState = 0;
      }
      */
      if (zeroCount > hallValuesStored / 2) {
        m_currentState = 0;
      } else {
        m_currentState = 1;
      }
      
      if (m_currentState == previousState) {
        return false;
      }
      return true;
    }

  private:
    int m_currentState;
    int m_previousReadings[hallValuesStored];
    int m_valuesStored;
    int m_inputPin;
};

/*-----GLOBAL VARIABLES----*/
HallSensor greenSensor = HallSensor(hallGreen);
HallSensor graySensor = HallSensor(hallGray);
HallSensor whiteSensor = HallSensor(hallWhite);
HallSensor *hallSensors[3];

MotorPhase phaseA = MotorPhase(aHigh, aLow);
MotorPhase phaseB = MotorPhase(bHigh, bLow);
MotorPhase phaseC = MotorPhase(cHigh, cLow);
MotorPhase *motorPhases[3];
/*-----------------*/

void readThrottle() {
  throttleRead = analogRead(throttleIn);

  if (throttleRead < lowerBound) {
  lowerBound = throttleRead;
  } else if (throttleRead > upperBound) {
    upperBound = throttleRead;
  }

  throttleRead = map(throttleRead, lowerBound, upperBound, 0, 255);

}

int selectDelay() {

  double pwm = (double) motorWrite;
  return ((motorWrite) * (-13.0) / (255.0)) + (13.0 + k); 
  
}

void ramp() { 
  //uncomment for ramp, millis() needs testing
  /*
  if (!timerStart) {
    timer = millis();
    timerStart = true;
  }
  if (millis() - timer >= selectDelay()){
    motorWrite += 1;
    timerStart = false;
  }
   */
   motorWrite = throttleRead;
}

void setup() {

  if (debugging == 1) {
    Serial.begin(9600);
  }
  
  pinMode(hallGreen, INPUT);
  pinMode(hallGray, INPUT);
  pinMode(hallWhite, INPUT);
  pinMode(aHigh, OUTPUT);
  pinMode(aLow, OUTPUT);
  pinMode(bHigh, OUTPUT);
  pinMode(bLow, OUTPUT);
  pinMode(cHigh, OUTPUT);
  pinMode(cLow, OUTPUT);
  pinMode(throttleIn, INPUT);

  hallSensors[0] = &graySensor;
  hallSensors[1] = &greenSensor;
  hallSensors[2] = &whiteSensor;

  motorPhases[0] = &phaseA;
  motorPhases[1] = &phaseB;
  motorPhases[2] = &phaseC;


  //Set Teensy PWM Frequency
  int freq = 10000;
  analogWriteFrequency(3, freq);
  analogWriteFrequency(4, freq);
  analogWriteFrequency(5, freq);
  analogWriteFrequency(6, freq);
  analogWriteFrequency(9, freq);
  analogWriteFrequency(10, freq);
  analogWriteResolution(8);

}

void loop() {
  
  readThrottle();
  motorWrite = throttleRead;
  /*
  if (throttleRead > motorWrite) {
    ramp();
   // ramp();
  } else {
    timerStart = false;
    motorWrite = throttleRead;
  }
  */

  
  for (int i = 0; i < 3; i++) {
    hallSensors[i]->determineState();
  }

  previousStep = currentStep;

  int nextState = -1;
  int i = 1;

 if (motorWrite < 5) {
  motorWrite = 0;
 }

 if (fullThrottle == 1) {
  motorWrite = 255;
 }
  
  if (hallSensors[0]->currentState() == 1 && hallSensors[1]->currentState() == 0 && hallSensors[2]->currentState() == 0) {
    nextState = 2;
  } else if (hallSensors[0]->currentState() == 1 && hallSensors[1]->currentState() == 1 && hallSensors[2]->currentState() == 0) {
    nextState = 3;
  } else if (hallSensors[0]->currentState() == 0 && hallSensors[1]->currentState() == 1 && hallSensors[2]->currentState() == 0) {
    nextState = 4;
  } else if (hallSensors[0]->currentState() == 0 && hallSensors[1]->currentState() == 1 && hallSensors[2]->currentState() == 1) {
    nextState = 5;
  } else if (hallSensors[0]->currentState() == 0 && hallSensors[1]->currentState() == 0 && hallSensors[2]->currentState() == 1) {
    nextState = 6;
  } else if (hallSensors[0]->currentState() == 1 && hallSensors[1]->currentState() == 0 && hallSensors[2]->currentState() == 1) {
    nextState = 1;
  } else {
    //weird combination of on/off sensors
  }  

  if (nextState == -1) {
    nextState = previous;
  } else {
    previous = nextState;
  }
  

  if (debugging == 1) {
    Serial.print("Motor Step: ");
    Serial.print(nextState);
    Serial.print(" PWM Write: ");
    Serial.println(motorWrite);
    delay(10);
  }


  switch (nextState) {
    case 1:
      //Serial.println("1");
      motorPhases[0]->writePWM(1);
      motorPhases[1]->writePWM(-1);
      motorPhases[2]->writePWM(0);
      break;
    case 2:
      //Serial.println("2");
      motorPhases[0]->writePWM(1);
      motorPhases[1]->writePWM(0);
      motorPhases[2]->writePWM(-1);
      break;
    case 3:
      //Serial.println("3");
      motorPhases[0]->writePWM(0);
      motorPhases[1]->writePWM(1);
      motorPhases[2]->writePWM(-1);
      break;
    case 4:
      //Serial.println("4");
      motorPhases[0]->writePWM(-1);
      motorPhases[1]->writePWM(1);
      motorPhases[2]->writePWM(0);
      break;
    case 5:
      //Serial.println("5");
      motorPhases[0]->writePWM(-1);
      motorPhases[1]->writePWM(0);
      motorPhases[2]->writePWM(1);
      break;
    case 6:
      //Serial.println("6");
      motorPhases[0]->writePWM(0);
      motorPhases[1]->writePWM(-1);
      motorPhases[2]->writePWM(1);
      break;
    case 0:
      break;
  }


}
