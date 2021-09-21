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

#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
//#include "EEPROMAnything.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//parameters
/////////////
int cooldownDelay = 60 * 60 * 4; // in sec, triggers automatically at most every 4h (defined in seconds)
int openingTime = 5500; //in ms
int closingTime = 5800; //in ms
boolean debug = true;
int debounceThreshold = 8; //number of consecutive reads on a state before registering the input

//pin definition
////////////////
unsigned int levelFloaterPin = 3;
unsigned int floaterDriverPin = 4;
unsigned int warningFloaterPin = 5;

unsigned int motorOpenPin = 12;
unsigned int motorClosePin = 10;
unsigned int motorDriverPin = 11;

unsigned int buttonBackPin = 6;
unsigned int buttonUpPin = A3;
unsigned int buttonForwardPin = A2;
unsigned int buttonDownPin = A1;

unsigned int buzzerPin = 2;
unsigned int ledPin = A0;

//convenient global variable
unsigned int ledState[] = {0, 0, 1, 0, 2}; // 0 = off, 1 = on, 2 = blink
unsigned int buttonState[] = {0, 0, 0, 0}; // 0 = no press, 1=pressed
unsigned int floaterState[] = {1, 0}; //1=water, 0=no water
unsigned int inputStatus[6];
int mooreState;
int cooldownTimer;
int lastState;

//stat keeping
int uptimeLoc = 0;
int opentimeLoc = 32;
int openCountLoc = 64;
int closeCountLoc = 72;
unsigned long uptimeCheck = 0;
unsigned long opentimeCheck = 0;
typedef enum{
  warning= 0,
  back   = 1,
  up     = 2,
  select = 3,
  down   = 4,
  full   = 5,
} input_t;


Adafruit_NeoPixel led(1, ledPin, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

void setup() {
  Serial.begin(115200);
  
  pinMode(levelFloaterPin, INPUT_PULLUP);
  pinMode(warningFloaterPin, INPUT_PULLUP);
  pinMode(floaterDriverPin, OUTPUT);
  pinMode(motorOpenPin, OUTPUT);
  pinMode(motorClosePin, OUTPUT);
  pinMode(motorDriverPin, OUTPUT);
  pinMode(buttonBackPin, INPUT_PULLUP);
  pinMode(buttonUpPin, INPUT_PULLUP);
  pinMode(buttonForwardPin, INPUT_PULLUP);
  pinMode(buttonDownPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  //set pin status
  led.begin();
  led.clear();
  led.show();
  digitalWrite(floaterDriverPin, HIGH);
  digitalWrite(motorDriverPin, LOW);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  oled.clearDisplay();

  //self test
  actionBeep();//buzzer
  ledSet(1, 0, 0, 0); del(200); //red
  ledSet(0, 0, 0, 0); del(200); //off
  ledSet(0, 0, 1, 0); del(200); //green
  ledSet(0, 0, 0, 0); del(200); //off
  ledSet(0, 0, 0, 1); del(200); //blue
  ledSet(0, 0, 0, 0);        //off
  oled.invertDisplay(true); oled.display(); delay(500);  oled.invertDisplay(false); oled.display();

}

void loop() {
  char output[128];
  static bool history = false;
  static int count = 0;
  output[0] = '\0';
  if(getState(full))
    sprintf(output, "full,");
  if(getState(back))
    sprintf(output, "%s back,", output);
  if(getState(up))
    sprintf(output, "%s up,", output);
  if(getState(select))
    sprintf(output, "%s select,", output);
  if(getState(down))
    sprintf(output, "%s down,", output);
  if(getState(warning))
    sprintf(output, "%s warning", output);
  sprintf(output, "%s %d", output, count);

  if(history != getState(back)){
    history = !history;
    count++;
  }

  
  //Serial.println(output);
  oled.clearDisplay();
  oled.setTextSize(2);             // Normal 1:1 pixel scale
  oled.setTextColor(SSD1306_WHITE);        // Draw white text
  oled.setCursor(0,0);             // Start at top-left corner
  oled.println(output);
  oled.display();
  //actionBeep();
  for(int i = 0; i < 4; i ++)
    inputScan();
}

void actionBeep() {
  digitalWrite(buzzerPin, HIGH);
  delay(2);
  digitalWrite(buzzerPin, LOW);
}

void ledSet(int red, int deprecated, int green, int blue) {
  led.setPixelColor(0, led.Color(red << 4, green << 4, blue << 4));
  led.show();
}

void del(int ms) {
  delay(ms);
}

//inputs with debounce function
void inputScan() {
  int pinList[] = {warningFloaterPin, buttonBackPin, buttonUpPin, buttonForwardPin, buttonDownPin};
  int i;
  for (i = 0; i < 5; i++) {
    if (digitalRead(pinList[i]) == LOW) {
      inputStatus[i]++;
    }
    else {
      inputStatus[i] /= 2;
    }
  }

  if (digitalRead(levelFloaterPin) == HIGH)
    inputStatus[i]++;
  else
    inputStatus[i] /= 2;
}

//get the state of a given input
//return true if input is active
bool getState(input_t source){
  return inputStatus[source] > debounceThreshold;
}
