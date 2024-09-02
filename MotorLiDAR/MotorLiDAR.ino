// For 6V N20 micro motor
const int in1Pin = 2;
const int in2Pin = 3;

const int motorSpeed = 100; // Max speed (0-255)
const unsigned long moveTime = 10000; // Time in milliseconds to move 5 cm (100 seconds)

bool platformRaised = false;


// For 12V DC motor
int E1 = 5;
int M1 = 6;

const int encoderAPin = 7;
const int encoderBPin = 8;

volatile long encoderValue = 0;
volatile bool lastAState = LOW;
volatile bool lastBState = LOW;
float PPR = 374 * 4 * 2.40;
int degree;

void setup()
{
  Serial.begin(9600);

  pinMode(M1, OUTPUT);
  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderAPin), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderBISR, CHANGE);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
}

void loop()
{
  // int value;
  // for(value = 0 ; value <= 255; value+=5)
  // {
  //   digitalWrite(M1, HIGH);
  //   analogWrite(E1, value);   //PWM Speed Control
  //   delay(30);
  // }
  
  lastAState = digitalRead(encoderAPin);
  lastBState = digitalRead(encoderBPin);
  digitalWrite(M1, HIGH);
  analogWrite(E1, 100);

  // Calculate the degree within the current revolution
  int rev = encoderValue / PPR;
  int position = encoderValue - rev * PPR;
  degree = (position * 360) / PPR;
  Serial.println(degree);
  
  //lowerPlatform();
  delay(5); 
}

void encoderAISR() {
  bool aState = digitalRead(encoderAPin);
  bool bState = digitalRead(encoderBPin);
  
  if (aState != lastAState) {
    if (aState == bState) {
      encoderValue++;
    } else {
      encoderValue--;
    }
  }
  lastAState = aState;
}

void encoderBISR() {
  bool aState = digitalRead(encoderAPin);
  bool bState = digitalRead(encoderBPin);
  
  if (bState != lastBState) {
    if (aState == bState) {
      encoderValue--;
    } else {
      encoderValue++;
    }
  }
  
  lastBState = bState;
}

void raisePlatform() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  delay(moveTime);
  stopMotor();
}

void lowerPlatform() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  delay(moveTime);
  stopMotor();
}

void stopMotor() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
}