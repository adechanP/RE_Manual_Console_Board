#include <Arduino.h>
#include <BluetoothSerial.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to configure it.
#endif
BluetoothSerial SerialBT;

//GPIO Assignments
///Buttons
  #define BTN_1 17
  #define BTN_2 16
  #define BTN_3 13
  #define BTN_4 15
  #define BTN_5 2
  #define BTN_6 12
  #define BTN_7 14
  #define ROTE1_BTN 34
  #define ROTE2_BTN 33
  #define ROTE3_BTN 25
  #define ROTE4_BTN 19

///Rotary Encoder
  #define ROTE1_A 36
  #define ROTE1_B 39
  #define ROTE2_A 35
  #define ROTE2_B 32
  #define ROTE3_A 27
  #define ROTE3_B 26
  #define ROTE4_A 5
  #define ROTE4_B 18

///Variables
  int aState1;
  int aState2;
  int aState3;
  int aState4;
  int alastState1;
  int alastState2;
  int alastState3;
  int alastState4;

void setup() {
  SerialBT.begin("ESP32_BTN_TEST"); //Bluetooth device name
  Serial.begin(115200);
  pinMode(BTN_1, INPUT);
  pinMode(BTN_2, INPUT);
  pinMode(BTN_3, INPUT);
  pinMode(BTN_4, INPUT);
  pinMode(BTN_5, INPUT);
  pinMode(BTN_6, INPUT);
  pinMode(BTN_7, INPUT);
  pinMode(ROTE1_BTN, INPUT);
  pinMode(ROTE2_BTN, INPUT);
  pinMode(ROTE3_BTN, INPUT);
  pinMode(ROTE4_BTN, INPUT);
  pinMode(ROTE1_A, INPUT);
  pinMode(ROTE1_B, INPUT);
  pinMode(ROTE2_A, INPUT);
  pinMode(ROTE2_B, INPUT);
  pinMode(ROTE3_A, INPUT);
  pinMode(ROTE3_B, INPUT);
  pinMode(ROTE4_A, INPUT);
  pinMode(ROTE4_B, INPUT);
  
  //Read initial state of rotary encoder
  alastState1 = digitalRead(ROTE1_A);
  alastState2 = digitalRead(ROTE2_A);
  alastState3 = digitalRead(ROTE3_A);
  alastState4 = digitalRead(ROTE4_A);  

  SerialBT.println("Bluetooth device is ready, waiting for connections...");
  }

void loop() {
  if (digitalRead(BTN_1) == HIGH) {
    Serial.println("Button 1 pressed");
    SerialBT.println("Button 1 pressed");
  }
  else if (digitalRead(BTN_2) == HIGH) {
    Serial.println("Button 2 pressed");
    SerialBT.println("Button 2 pressed");
  }
  else if (digitalRead(BTN_3) == HIGH) {
    Serial.println("Button 3 pressed");
    SerialBT.println("Button 3 pressed");
  }
  else if (digitalRead(BTN_4) == HIGH) {
    Serial.println("Button 4 pressed");
    SerialBT.println("Button 4 pressed");
  }
  else if (digitalRead(BTN_5) == HIGH) {
    Serial.println("Button 5 pressed");
    SerialBT.println("Button 5 pressed");
  }
  else if (digitalRead(BTN_6) == HIGH) {
    Serial.println("Button 6 pressed");
    SerialBT.println("Button 6 pressed");
  }
  else if (digitalRead(BTN_7) == HIGH) {
    Serial.println("Button 7 pressed");
    SerialBT.println("Button 7 pressed");
  }
  else if (digitalRead(ROTE1_BTN) == HIGH) {
    Serial.println("Rotary 1 button pressed");
    SerialBT.println("Rotary 1 button pressed");
  }
  else if (digitalRead(ROTE2_BTN) == HIGH) {
    Serial.println("Rotary 2 button pressed");
    SerialBT.println("Rotary 2 button pressed");
  }
  else if (digitalRead(ROTE3_BTN) == HIGH) {
    Serial.println("Rotary 3 button pressed");
    SerialBT.println("Rotary 3 button pressed");
  }
  else if (digitalRead(ROTE4_BTN) == HIGH) {
    Serial.println("Rotary 4 button pressed");
    SerialBT.println("Rotary 4 button pressed");
  }
  else {
    //Read state of rotary encoder
    aState1 = digitalRead(ROTE1_A);
    aState2 = digitalRead(ROTE2_A);
    aState3 = digitalRead(ROTE3_A);
    aState4 = digitalRead(ROTE4_A);
    //Check if rotary encoder has been turned
    if (aState1 != alastState1) {
      if (digitalRead(ROTE1_B) != aState1) {
        Serial.println("Rotary 1 turned clockwise");
        SerialBT.println("Rotary 1 turned clockwise");
      }
      else {
        Serial.println("Rotary 1 turned counterclockwise");
        SerialBT.println("Rotary 1 turned counterclockwise");
      }
    }
    if (aState2 != alastState2) {
      if (digitalRead(ROTE2_B) != aState2) {
        Serial.println("Rotary 2 turned clockwise");
        SerialBT.println("Rotary 2 turned clockwise");
      }
      else {
        Serial.println("Rotary 2 turned counterclockwise");
        SerialBT.println("Rotary 2 turned counterclockwise");
      }
    }
    if (aState3 != alastState3) {
      if (digitalRead(ROTE3_B) != aState3) {
        Serial.println("Rotary 3 turned clockwise");
        SerialBT.println("Rotary 3 turned clockwise");
      }
      else {
        Serial.println("Rotary 3 turned counterclockwise");
        SerialBT.println("Rotary 3 turned counterclockwise");
      }
    }
    if (aState4 != alastState4) {
      if (digitalRead(ROTE4_B) != aState4) {
        Serial.println("Rotary 4 turned clockwise");
        SerialBT.println("Rotary 4 turned clockwise");
      }
      else {
        Serial.println("Rotary 4 turned counterclockwise");
        SerialBT.println("Rotary 4 turned counterclockwise");
      }
    }
    //Update last state of rotary encoder
    alastState1 = aState1;
    alastState2 = aState2;
    alastState3 = aState3;
    alastState4 = aState4;
  }
}
