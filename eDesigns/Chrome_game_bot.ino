#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>                         
const int soundPin = 4;
const int resetButtonPin = 2;
const int pressAngle = 100;
const int restAngle = 40;
int pressCount = 0;
bool triggered = false;

Servo myservo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(soundPin, INPUT);
  pinMode(resetButtonPin, INPUT);
  myservo.attach(9);
  myservo.write(restAngle);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello!");
  lcd.setCursor(0, 1);
  lcd.print("Presses: 0");
}

void loop() {
  int soundState = digitalRead(soundPin);

  if (soundState == HIGH && !triggered) {
    pressKey();
    pressCount++;
    updateDisplay();
    triggered = true;
  }

  if (soundState == LOW && triggered) {
    myservo.write(restAngle);
    triggered = false;
    delay(300);
  }

  if (digitalRead(resetButtonPin) == HIGH) {
    pressCount = 0;
    updateDisplay();
    delay(200);
  }

  //delay(100);
}

void pressKey() {
  myservo.write(pressAngle);
  delay(300);
  myservo.write(restAngle);
  delay(500);
}

void updateDisplay() {
  lcd.setCursor(10, 1);
  lcd.print(" ");
  lcd.setCursor(10, 1);
  lcd.print(pressCount);
  

}