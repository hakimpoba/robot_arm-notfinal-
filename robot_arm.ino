#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#include <Keypad.h>
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6};//connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2};  //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS );

char holdKey;
unsigned long t_hold;
#define SERVOMIN  125 //minimum
#define SERVOMAX  400 //maximum 575
uint8_t servonum = 0;

void setup(){
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50); // Set the PWM frequency to 50Hz
initial();

}
 
void loop(){
  char key = keypad.getKey();
 
   if (key){
     holdKey = key;
     Serial.println(key);
   }
 
   if (keypad.getState() == HOLD) {
      if ((millis() - t_hold) > 5 ) {
          switch (holdKey) {
               case '1':
                  incrementServo(servonum, 2);
                  Serial.println("servo1++");
                  break;
                case '2':
                  incrementServo(servonum, -2);
                  Serial.println("servo1--");
                  break;
                case '5':
                  incrementServo(servonum + 1, 2);
                  Serial.println("servo2++");
                  break;
                case '4':
                  incrementServo(servonum + 1, -2);
                   Serial.println("servo2--");
                  break;
                case '7':
                  incrementServo(servonum + 2, 2);
                   Serial.println("servo3++");
                  break;
                case '8':
                  incrementServo(servonum + 2, -2);
                  Serial.println("servo3--");
                  break;
                case '*':
                  incrementServo(servonum + 3, 1);
                  Serial.println("servo4++");
                  break;
                case '0':
                  incrementServo(servonum + 3, -1);
                  Serial.println("servo4--");
                  break;
          }
          t_hold = millis();
      }
   }
}
void initial(){
  pwm.setPWM(0, 0, 300);
  delay(1000);
  pwm.setPWM(1, 0, 200);
  delay(1000);
  pwm.setPWM(2, 0, 200);
  delay(1000);
  pwm.setPWM(3, 0, 350);
}
void incrementServo(uint8_t servoNum, int8_t increment) {
  static int16_t servoPositions[4] = {230, 200, 200, 350};
  servoPositions[servoNum] += increment;

  // Limit the servo position within the allowed range
  servoPositions[servoNum] = constrain(servoPositions[servoNum], SERVOMIN, SERVOMAX);

  // Set the servo position
  pwm.setPWM(servoNum, 0, servoPositions[servoNum]);
}
