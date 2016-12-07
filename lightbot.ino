#include <MotorShield.h>
#include <Ultrasonic.h>

// A0   :  Motor Current A
// A1   :  Motor Current B
// A2   :  LDR Right
// A3   :  LDR Central
// A4   :  LDR Left
// A5   :
// D0   :  Serial Tx
// D1   :  Serial Rx
// D2   :  Switch
// D3~  :  Motor PWM A
// D4   :  Red LED
// D5~  :  Green LED
// D6~  :  Ultrasound Echo
// D7   :  Ultrasound Trigger
// D8   :  Motor Brake B
// D9~  :  Motor Brake A
// D10~ :  Yellow LED
// D11~ :  Motor PWM B
// D12  :  Motor Dir A
// D13  :  Motor Dir B

//Light Sensor Parameters
int leftValue = 0;
int centralValue = 0;
int rightValue = 0;
int rightPin = A5;
int centralPin = A4;
int leftPin = A3;
int sensitivity = 100; //Only follow single LDR if above this value, else follow two
int threshold = 0; //Lowest light level it will follow

//Ultrasonic Parameters
int trigPin = 7;
int echoPin = 6;
long distance = 0;
long minDistance = 10;
Ultrasonic ultrasonic(trigPin,echoPin);

//Motor Parameters
int maxSpeed = 255;
MS_DCMotor motorR(MOTOR_A);
MS_DCMotor motorL(MOTOR_B);

//LED Parameters
int redPin = 4;
int greenPin = 5;
int yellowPin = 10;
long yellowBlinkRate = 250;
long yellowPrevious = 0;
unsigned long yellowCurrent = 0;
int yellowState = HIGH;

//Switch Parameters
int switchPin = 2;

void setup() {
  pinMode(redPin,OUTPUT);
  pinMode(greenPin,OUTPUT);
  pinMode(yellowPin,OUTPUT);
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(yellowPin, yellowState);

  motorL.run(BRAKE);
  motorL.setSpeed(maxSpeed);
  motorR.run(BRAKE);
  motorR.setSpeed(maxSpeed);

  Serial.begin(9600);

}

void loop() {

  if(digitalRead(switchPin) == HIGH){
    motorL.run(BRAKE);
    motorR.run(BRAKE);
    digitalWrite(redPin,LOW);
    digitalWrite(greenPin,LOW);
    yellowCurrent = millis();
    if(yellowCurrent - yellowPrevious > yellowBlinkRate) {
      if(yellowState == HIGH){
        yellowState = LOW;
      }
      else{
        yellowState = HIGH;
      }
      yellowPrevious = yellowCurrent; 
      digitalWrite(yellowPin,yellowState);
    }
  }
  else{

    yellowState = HIGH;
    digitalWrite(yellowPin,yellowState);
    //Collect Values
    leftValue = analogRead(leftPin);
    centralValue = analogRead(centralPin);
    rightValue = analogRead(rightPin);
    distance = ultrasonic.Ranging(CM);

    //Adjust values
    leftValue = 1023 - leftValue;
    centralValue = 1023 - centralValue;
    rightValue = 1023 - rightValue;

    Serial.print(leftValue);
    Serial.print(" - ");
    Serial.print(centralValue);
    Serial.print(" - ");
    Serial.println(rightValue);

    digitalWrite(yellowPin,LOW);
    if(distance > minDistance){
      digitalWrite(redPin,LOW);
      digitalWrite(greenPin,HIGH);

      if((leftValue > (centralValue+sensitivity)) & (leftValue > (rightValue+sensitivity))){
        if(leftValue > threshold){
          motorL.run(BACKWARD|RELEASE);
          motorR.run(FORWARD|RELEASE);
        }
        else{
          motorL.run(BRAKE);
          motorR.run(BRAKE);
        }
      }
      else if((centralValue > (leftValue+sensitivity)) & (centralValue > (rightValue+sensitivity))){
        if(centralValue > threshold){
          motorL.run(FORWARD|RELEASE);
          motorR.run(FORWARD|RELEASE);
        }
        else{
          motorL.run(BRAKE);
          motorR.run(BRAKE);
        }
      }
      else if((rightValue > (centralValue+sensitivity)) & (rightValue > (leftValue+sensitivity))){
        if(rightValue > threshold){;
          motorL.run(FORWARD|RELEASE);
          motorR.run(BACKWARD|RELEASE);
        }
        else{
          motorL.run(BRAKE);
          motorR.run(BRAKE);
        }
      }
      else{
        if((leftValue + centralValue)>(rightValue + centralValue)){
          if((leftValue + centralValue) > (threshold*2)){;
            motorL.run(BACKWARD|RELEASE);
            motorR.run(FORWARD|RELEASE);
          }
          else{
            motorL.run(BRAKE);
            motorR.run(BRAKE);
          }
        }
        else if((rightValue + centralValue)>(leftValue + centralValue)){
          if((leftValue + centralValue) > (threshold*2)){;
            motorL.run(FORWARD|RELEASE);
            motorR.run(BACKWARD|RELEASE);
          }
          else{
            motorL.run(BRAKE);
            motorR.run(BRAKE);
          }
        }
        else{
          //All three are the same. Do nothing
          motorL.run(BRAKE);
          motorR.run(BRAKE);
        }
      }
    }
    else{
      digitalWrite(redPin,HIGH);
      digitalWrite(greenPin,LOW);
      motorL.run(BRAKE);
      motorR.run(BRAKE);
    }

  }
  //delay(10);
}
