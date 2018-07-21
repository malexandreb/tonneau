//2018-07-16 MAB and SAB
//sofware to control the water tank at rte de chesalles 90
//2 sensor:
//floater to detect high water - simple, and enough
//floater to detect water TOO HIGH
//1 actuator: motor to open and close the valve, no sensor on state of motor
//3 buttons
//enable/disable - disable state for maintenance purposes
//spritz (short opening during disable state)
//force on -- for manually triggering refill
//4 leds
// red = disable
// orange is spritz
// green for ready to refill (timer up or forced fill)
// blue = refilling
//1 buzzer for water too high alert and button press feedback

//parameters
/////////////
int cooldownDelay = 60 * 60 * 6; // in sec, triggers automatically at most every 6h (defined in seconds)
int openingTime = 2500; //in ms
int closingTime = 2550; //in ms

//pin definition
////////////////
unsigned int levelFloaterPin = 12;
unsigned int warningFloaterPin = 10;
unsigned int motorOpenPin = 8;
unsigned int motorClosePin = 6;
unsigned int buttonDisablePin = A1;
unsigned int buttonSpritzPin = A4;
unsigned int buttonForcePin = 2;
unsigned int redLedPin = A0;
unsigned int orangeLedPin = A3;
unsigned int greenLedPin = 3;
unsigned int blueLedPin = 4;
unsigned int buzzerPin = A2;

//convenient global variable
unsigned int ledState[] = {0, 0, 1, 0}; // 0 = off, 1 = on, 2 = blink
unsigned int ledPin[] = {redLedPin, orangeLedPin, greenLedPin, blueLedPin};
unsigned int buttonState[] = {0, 0, 0}; // 0 = no press, 1=pressed
unsigned int floaterState[] = {1, 0}; //1=water, 0=no water
int mooreState;
int cooldownTimer;

void setup() {
  pinMode(levelFloaterPin, INPUT_PULLUP);
  pinMode(warningFloaterPin, INPUT_PULLUP);
  pinMode(motorOpenPin, OUTPUT);
  pinMode(motorClosePin, OUTPUT);
  pinMode(buttonDisablePin, INPUT_PULLUP);
  pinMode(buttonSpritzPin, INPUT_PULLUP);
  pinMode(buttonForcePin, INPUT_PULLUP);
  pinMode(redLedPin, OUTPUT);
  pinMode(orangeLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  nodeStart();

}

void loop() {
  //behavior simply depends of the state on a Moore state machine
  //this is a simple enough controller, and that makes valitation easier
  switch (mooreState) {
    case 0: nodeIdle(); break;
    case 1: nodeCheckLevel(); break;
    case 2: nodeOpenValve(); break;
    case 3: nodeFilling(); break;
    case 4: nodeCloseValve(); break;
    case 5: nodeResetTimer(); break;
    case 6: nodeCloseToDisable(); break;
    case 7: nodeDisabled(); break;
    case 8: nodeSpritzOpen(); break;
    case 9: nodeSpritzClose(); break;
    default: //should never reach this state
      while (true) {
        ledSet(2, 2, 0, 0);
        errorBeep();
        del(5000);
      }
  }
}

///////////////////////////
///// STATE FUNCTIONS /////
///////////////////////////
//refer to state diagram for each state function

//simple node, just used once at the very beginning
void nodeStart() {
  //start test procedure
  ledSet(2, 2, 2, 2);
  for (int i = 0; i < 4; i++) {
    del(1000);
    buttonBeep();
  }
  ledSet(0, 0, 0, 0);
  cooldownTimer = cooldownDelay;
  mooreState = 0; //set state machine to idle state
}

//STATE 0: idle, monitors buttons, waits for cooldown timer to expire
void nodeIdle() {
  ledSet(0, 1, 0, 0);
  while (cooldownTimer > 0) {
    cooldownTimer--;
    if (buttonPressed()) {
      if (buttonDisable()) {
        mooreState = 7; // to disabled state
        buttonClear();
        return;
      }
      if (buttonForce()) {
        mooreState = 1; //to check level state
        ledSet(0, 0, 1, 0); //force is solid green
        buttonClear();
        return;
      }
      //spritz does nothing here
      errorBeep();
      buttonClear();
    }
    del(1000);
  }
  //time up, going to next state
  ledSet(0, 0, 2, 0); //timeout is blinking green
  mooreState = 1; // to check level state
  return;
}

//STATE 1, check level and wait for water to drop if level is up
void nodeCheckLevel() {
  //ledSet taken care by the previous state

  //stay in state 1 if level is high
  //go to state 2 if level is low
  if (!tankFull()) {
    mooreState = 2;
  }

  //go to state 7 if disable button is pressed
  if (buttonPressed()) {
    if (buttonDisable()) {
      mooreState = 7;
      buttonClear();
      return;
    }
    errorBeep(); //force and spritz make no sense here
    buttonClear();
  }

  del(1000);
}

//STATE 2, open the valve
void nodeOpenValve() {
  actionBeep();
  ledSet(0, 0, 2, 1);
  del(10000); // wait for 10 sec for things to stabilize
  openValve();
  mooreState = 3; //go to state 3 after valve is opened
}

//STATE 3, filling, valve is open, monitoring floater
void nodeFilling() {
  ledSet(0,0,0,1);
  //stay in state 3 if level is low
  //go to state 4 if level is high
  if (tankFull()) {
    del(10000); //splash guard
    if (tankFull()) {
      mooreState = 4; //close the valve since it is full
    }
  }
  //go to state 6 if disable button is pressed
}

//STATE 4, done filling, tank full event detected, closing the valve
void nodeCloseValve() {
  ledSet(2,0,0,1);
  del(10000); //make sure the floater is floating
  //go to state 5 after valve is closed
}

//STATE 5, fill process done, resetting the cooldown timer and back to idle state
void nodeResetTimer() {
  cooldownTimer = cooldownDelay; //reset timer
  mooreState = 0; //go to state idle
}

//STATE 6, disable button pressed during fill, closing valve and going to disable mode
void nodeCloseToDisable() {
  //go to state 7 after valve is closed
}

//STATE 7, autoFill disabled until disable button pressed again.
void nodeDisabled() {
  ledSet(1, 0, 0, 0); //set led
  //infinite loop here
  while (true) {
    if (buttonPressed()) {
      if (buttonForce()) {
        mooreState = 1; // to check level state
        buttonClear();
        return;
      }
      if (buttonSpritz()) {
        mooreState = 8; //go to spritz open
        buttonClear();
        return;
      }
      //disable does nothing here
      errorBeep();
      buttonClear();
    }
    del(1000);
  }
}

//STATE 8, partial open of the valve to give a little bit of water during tank maintenance
void nodeSpritzOpen() {
  //go to state 9 after partial open of valve
}

//STATE 9, closing vlave after partial open
void nodeSpritzClose() {
  //go to state 7 after partial close of valve
}


////////////////////////////
///// HELPER FUNCTIONS /////
////////////////////////////

//DELAY//
//replacing busy wait with something more usefull
void del(unsigned long ms) {
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  while (!((unsigned long)(currentMillis - previousMillis) >= ms)) {
    currentMillis = millis();
    //put monitoring code here
    showLed();
    getButtonState();
  }
}

//MOTOR//
/////////

void openValve() {
  //make green and blue led blink
  ledSet(0, 0, 2, 2);
  del(1000);
  actionBeep();
  del(1000);
  digitalWrite(motorOpenPin, HIGH);
  del(openingTime);
  digitalWrite(motorOpenPin, LOW);
}

void closeValve() {
  //make red and blue led blink
  ledSet(2, 0, 0, 2);
  del(1000);
  actionBeep();
  del(1000);
  digitalWrite(motorClosePin, HIGH);
  del(closingTime);
  digitalWrite(motorClosePin, LOW);
}

/////LED////
////////////
void showLed() {
  for (int i = 0; i < 4; i++) {
    switch (ledState[i]) {
      case 0 : digitalWrite(ledPin[i], LOW); break;
      case 1 : digitalWrite(ledPin[i], HIGH); break;
      case 2 : digitalWrite(ledPin[i], blinkState()); break;
      default: break;
    }
  }
}

void ledSet(int red, int orange, int green, int blue) {
  ledState[0] = red;
  ledState[1] = orange;
  ledState[2] = green;
  ledState[3] = blue;
  showLed();
}

boolean blinkState() {
  return (millis() / 500) % 2;
}

//Buttons//
///////////

//returns 0 if no button pressed.
//returns 1 if disable button pressed
//returns 2 if spritz button pressed
//returns 3 if force button pressed
//lowest button number has priority if multiple button pressed
int getButtonState() {
  if (digitalRead(buttonDisablePin) == LOW) {
    buttonState[0] = 1;
    buttonBeep();
    return 1;
  }
  if (digitalRead(buttonSpritzPin) == LOW) {
    buttonState[1] = 1;
    buttonBeep();
    return 2;
  }
  if (digitalRead(buttonForcePin) == LOW) {
    buttonState[2] = 1;
    buttonBeep();
    return 3;
  }
  return 0;
}

//returns true if any of the button has been pressed
boolean buttonPressed() {
  return buttonState[0] + buttonState[1] + buttonState[2];
}

boolean buttonDisable() {
  return buttonState[0];
}
boolean buttonSpritz() {
  return buttonState[1];
}
boolean buttonForce() {
  return buttonState[2];
}

//resets all buttons to unpressed state
void buttonClear() {
  buttonState[0] = 0;
  buttonState[1] = 0;
  buttonState[2] = 0;
}

//Buzzer//
//////////
void buttonBeep() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(5);
    digitalWrite(buzzerPin, LOW);
    delay(10);
  }
}

void errorBeep() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(buzzerPin, HIGH);
    del(300);
    digitalWrite(buzzerPin, LOW);
    del(100);
  }
}

void actionBeep() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(2);
    digitalWrite(buzzerPin, LOW);
    del(20);
  }
}

//floaters//
////////////

//return true if the tank is detected as full
boolean tankFull() {
  return digitalRead(levelFloaterPin) == HIGH;
}

//retun true if overfill sensor triggered
boolean tankOverfullWarning() {
  return digitalRead(warningFloaterPin) == HIGH;
}
