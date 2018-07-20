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


//pin definition
////////////////
int levelFloaterPin = ;
int warningFloaterPin = ;
int motorOpenPin = ;
int motorClosePin = ;
int buttonDisablePin = ;
int buttonSpritzPin = ;
int buttonForcePin = ;
int redLedPin = ;
int orangeLedPin = ;
int greenLedPin = ;
int blueLedPin = ;
int buzzerPin = ;

//convenient global variable
int ledState[4];

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

void buttonBeep(){
  for(int i = 0; i<2; i++){
    digitalWrite(buzzerPin, HIGH); 
    delay(5);
    digitalWrite(buzzerPin, LOW);
    delay(10);
  }
}

boolean blinkState(){
  return (millis()/500) % 2;
}
