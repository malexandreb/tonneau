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
int intervall = 60*60*6; //triggers automatically at most every 6h (defined in seconds)
int openingTime = 2500;
int closingTime = 2550;

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
unsigned int ledState[]={0,0,1,0}; // 0 = off, 1 = on, 2 = blink
unsigned int ledPin[]={redLedPin, orangeLedPin, greenLedPin, blueLedPin};
unsigned int buttonState[] = {0,0,0};

void setup(){
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
}

void loop(){
  
}

//DELAY//
//replacing busy wait with something more usefull
void del(unsigned long ms){
 unsigned long previousMillis = millis();
 unsigned long currentMillis = millis();
 while(!((unsigned long)(currentMillis - previousMillis) >= ms)){
  currentMillis = millis();
  //put monitoring code here
  showLed();
  getButtonState();
 } 
}

//MOTOR//
/////////

void openValve(){
  //make green and blue led blink
  ledSet(0,0,2,2);
  del(1000);
  actionBeep();
  del(1000);
  digitalWrite(motorOpenPin, HIGH);
  del(openingTime);
  digitalWrite(motorOpenPin, LOW);
}

void closeValve(){
  //make red and blue led blink
  ledSet(2,0,0,2);
  del(1000);
  actionBeep();
  del(1000);
  digitalWrite(motorClosePin, HIGH);
  del(closingTime);
  digitalWrite(motorClosePin, LOW);
}

/////LED////
////////////
void showLed(){
  for(int i=0; i<4; i++){
    switch(ledState[i]){
      case 0 : digitalWrite(ledPin[i], LOW); break;
      case 1 : digitalWrite(ledPin[i], HIGH); break;
      case 2 : digitalWrite(ledPin[i], blinkState()); break;
      default: break;
    }
  }
}

void ledSet(int red, int orange, int green, int blue){
  ledState[0]=red;
  ledState[1]=orange;
  ledState[2]=green;
  ledState[3]=blue;
}

boolean blinkState(){
  return (millis()/500) % 2;
}

//Buttons//
///////////

//returns 0 if no button pressed.
//returns 1 if disable button pressed
//returns 2 if spritz button pressed
//returns 3 if force button pressed
//lowest button number has priority if multiple button pressed
int getButtonState(){
  if(digitalRead(buttonDisablePin) == LOW){
    buttonState[0] = 1;
    return 1;
  }
  if(digitalRead(buttonSpritzPin) == LOW){
    buttonState[1] = 1;
    return 2;
  }
  if(digitalRead(buttonForcePin) == LOW){
    buttonState[2] = 1;
    return 3; 
  } 
  return 0;
}

void buttonBeep(){
  for(int i = 0; i<2; i++){
    digitalWrite(buzzerPin, HIGH); 
    delay(5);
    digitalWrite(buzzerPin, LOW);
    delay(10);
  }
}

void actionBeep(){
  for(int i = 0; i<10; i++){
    digitalWrite(buzzerPin, HIGH); 
    delay(2);
    digitalWrite(buzzerPin, LOW);
    delay(2);
  }
}
