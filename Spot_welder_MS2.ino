// Spot solder
// Generates 2 pulses (T1,T2) spaced with some delay (D1, D2) when button is pressed
// default timings can be changed (not saved) using usb/serial commands like t1=xxx, t2=xxx, d1=xxx, d2=xxx
// timings are in millis sec (mutiple of 10 ms because relay starts at zero crossing)
// pressure of each electrode  on the plate is about 400gr

// to do: add code for rotary encoder (currently added but not used)
// add a display
// add a way to save the parameters in eeprom


// the display has 8 lines with 16 coloms (of 8X8 pixels) when u8x8 lib is used
// if we display in X10 ms, T1 tequires max 2 char + blank, d1 max 3+ 1, T2 max 3+1 so a total of 11 char
// then we can't use 2X font
// if we use 2X2 font, we could display
//    Pre =xxx (in ms)
//    Wait=xxx
//    Weld=XXX
//    Blank, Set Pre, Set Wait, Set Weld, Save

// We need a State = Blank, Set Pre, Set Wait, Set Weld, Save
// we also need a flag that say that we are editing or not
// when we rotate the button, 
//      If editing is false, then State change from Blank, Set Pre, Set Wait, Set Weld, Save
//      If editing is true, then we change pre, wait, weld (within the limits and with a step of 10)
// When we press the button
//      If state = blank, no action
//      if state = Save, save the current values pre, wait, weld and then set state on blank
//      if state = pre,wait,weld, change the flag editing
// In each loop, if State or editing change, we update the screen
//     The first 3 lines (X2) are fixed with Pre, Wait, Weld text and value; Still if editing is true, 
//                       then the line related to the state is inverted
//     The last lines, display the state

#include <Arduino.h>
//#include <U8g2lib.h>
#include <U8x8lib.h>
//#include <SPI.h>
#include <Wire.h>
// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include "Bounce2.h"
#include "RotaryEncoder.h"
#include <EEPROM.h>

//Set default delays
#define T1 20
#define DELAY_T1_T2 100
#define T2 60
#define DELAY_NEXT 1000

// Set pins for welding
#define LED_PIN 13
#define BUTTON_PIN A2 // Connected to buuton to request the welding; there must be a resistor (e.g. 2k between this pin and grnd)
                      // other pin of button is connected to 5V
#define RELAY_PIN A0  // other pin of relay is connected to Grnd

// Set pins for rotary encoder
// the rotary I used has also a connection to 5V and to Grnd.
#define PIN_IN1 3 // do not change those pins because it is the only one that support interrupt on AVR328P
#define PIN_IN2 2 // do not change those pins because it is the only one that support interrupt on AVR328P
                  // invert the 2 pins 3 and 2 here in the config if direction of rotation is inverted
#define PIN_ROTARY_BUTTON 4

// Pins used for I2C display (use hardware I2C)
// SDA = A4
// SCL = A5

// State values
#define BLANK 0
#define PRE 1
#define WAIT 2
#define WELD 3
#define SAVE 4

char stateName[][16] = {" "," Select "," Select "," Select ", "  Save  " };
char stateNameEditing[][16] = {" ","Changing","Changing","Changing", "  Save  " };



int8_t state = BLANK;
bool editing = false;
int8_t prevState = PRE; // set another value than state to force a display in first loop
bool prevEditing = false;
bool updateDisplay = true; 


U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button button = Bounce2::Button();
Bounce2::Button rotaryButton = Bounce2::Button();

// variables to store the welding delay; set to default values
int16_t t1 = T1;
int16_t d1 = DELAY_T1_T2;
int16_t t2 = T2;
int16_t d2 = DELAY_NEXT;

// SET A VARIABLE TO STORE THE LED STATE
int ledState = LOW;


// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}


void fillScreen(){
  if ((state != prevState) or (editing != prevEditing) or (updateDisplay)) {
    updateDisplay = false;
    prevState = state;
    prevEditing = editing;

    u8x8.clear();
    char valS[16];
    u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);

    if ( (state ==  PRE)  ) u8x8.inverse(); else u8x8.noInverse();
    u8x8.drawString(0, 0, "Pre  ");
    if ( (state ==  PRE) and (editing) ) u8x8.inverse(); else u8x8.noInverse();
    sprintf(valS,"%3d",t1);
    u8x8.drawString(10, 0, valS);//u8x8.print(t1);

    if ( (state ==  WAIT)  ) u8x8.inverse(); else u8x8.noInverse();
    u8x8.drawString(0, 2, "Wait ");
    if ( (state ==  WAIT) and (editing) ) u8x8.inverse(); else u8x8.noInverse();
    sprintf(valS,"%3d",d1);
    u8x8.drawString(10, 2, valS);//u8x8.print(d1);

    if ( (state ==  WELD)  ) u8x8.inverse(); else u8x8.noInverse();
    u8x8.drawString(0, 4, "Weld ");
    if ( (state ==  WELD) and (editing) ) u8x8.inverse(); else u8x8.noInverse();
    sprintf(valS,"%3d",t2);
    u8x8.drawString(10, 4, valS);//u8x8.print(t2);

    if ( editing) {
      u8x8.inverse();
      u8x8.drawString(0,6,stateNameEditing[state]);
    }else {
      u8x8.noInverse();
      u8x8.drawString(0,6,stateName[state]);
    }
  }
}

void getConfig(){
  if (EEPROM.read(0) != 0xAA){
    // variables to store the welding delay; set to default values when EEPROM is not preloaded
    t1 = T1;
    d1 = DELAY_T1_T2;
    t2 = T2;
    d2 = DELAY_NEXT;
  } else {
    // variables to store the welding delay; set to default values
    t1 = (int16_t)(EEPROM.read(1) << 8) | (EEPROM.read(2) ) ;
    d1 = (int16_t)(EEPROM.read(3) << 8) | (EEPROM.read(4) ) ;
    t2 = (int16_t)(EEPROM.read(5) << 8) | (EEPROM.read(6) );
    d2 = (int16_t)(EEPROM.read(7) << 8) | (EEPROM.read(8) );
  }
}

void saveConfig(){
  u8x8.clear();
  u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
  u8x8.drawString(0, 2, "Saving!");
  EEPROM.update(0,0XAA); // first byte to identify that eeprom contains some data
  EEPROM.update(1, (uint8_t) (t1>>8)); //upper part of t1
  EEPROM.update(2, (uint8_t) t1);    //lower part of t1
  EEPROM.update(3, (uint8_t) (d1>>8)); //upper part of d1
  EEPROM.update(4, (uint8_t) (d1));    //lower part of d1
  EEPROM.update(5, (uint8_t) (t2>>8)); //upper part of t2
  EEPROM.update(6, (uint8_t) (t2));    //lower part of t2
  EEPROM.update(7, (uint8_t) (d2>>8)); //upper part of d2
  EEPROM.update(8, (uint8_t) (d2));    //lower part of d2
  delay(1000);
  u8x8.drawString(0, 4, "   Done ");
  delay(1000);
  state=BLANK; // return to blank
  updateDisplay = true; //force to display again the screen even if state/editing did not changed
}

void setup() {

  getConfig();
  // BUTTON SETUP 
  
  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR BUTTON HAS AN INTERNAL PULL-UP
  // button.attach( BUTTON_PIN ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  // 2) IF YOUR BUTTON USES AN EXTERNAL PULL-UP
  button.attach( BUTTON_PIN, INPUT_PULLUP); // USE internal PULL-UP
  rotaryButton.attach( PIN_ROTARY_BUTTON, INPUT_PULLUP ); // USE internal PULL-UP
  

  // DEBOUNCE INTERVAL IN MILLISECONDS
  button.interval(5);
  rotaryButton.interval(5);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  button.setPressedState(LOW);
  rotaryButton.setPressedState(LOW);
  
  // LED SETUP
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,ledState);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Serial.begin(115200);

  setupLcd();
  // setup the rotary encoder functionality

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  ///encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
}

uint8_t buffer[80];
uint8_t bufferCount = 0;


const byte numChars = 254;
char receivedChars[numChars];

boolean newData = false;

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
      ndx = numChars - 1;
      }
  }
  else {
    receivedChars[ndx] = '\n'; // terminate the string
    ndx = 0;
    newData = true;
    }
 }
}


#include <string.h>

void processCmd() {
  char * key_p;
  char * value_p;
  int valueInt = 0;
  if (newData == false) {
    return;
  }
  Serial.print("Received char=");Serial.println(receivedChars); 
  newData = false;
  key_p = strtok(receivedChars, "=");    // Everything up to the '=' is the  cmd
  value_p = strtok(NULL, "\n");  // Everything else is the value
  Serial.print("Key=");Serial.println(key_p); 
  Serial.print("value=");Serial.println(value_p); 
  
  if ((key_p != NULL) && (value_p != NULL)){
    valueInt = atoi(value_p);
    if (valueInt < 10 or valueInt> 3000) {
      Serial.println("The value must be in range 10 /1000");
      return;
    }
    valueInt = (valueInt / 10 ) * 10;
    if (( strcmp(key_p, "T1")==0) or ( strcmp(key_p, "t1"))==0 ){
      t1 = valueInt;
    } else if (( strcmp(key_p, "T2")==0) or ( strcmp(key_p, "t2")==0) ){
      t2 = valueInt;
    } else if (( strcmp(key_p, "D1")==0) or ( strcmp(key_p, "d1")==0) ){
      d1 = valueInt;
    } else if (( strcmp(key_p, "D2")==0) or ( strcmp(key_p, "d2")==0) ){
      d2 = valueInt;
    } else {
      Serial.println("error in the syntax of the command");
      Serial.print("Key=");Serial.println(key_p);
      Serial.print("Value=");Serial.println(value_p);
      return;         
    }
    Serial.println("New parameters:");
    Serial.print("T1="); Serial.println(t1);
    Serial.print("DELAY="); Serial.println(d1);
    Serial.print("T2="); Serial.println(t2);
    Serial.print("WAIT="); Serial.println(d2); 
  }
}

void performWelding(){ // generate the pulses for welding    
    if (editing) return; // skip when we are in editing mode 

    u8x8.clear();
    u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
    u8x8.drawString(0, 2, "Welding!");
    
    // TOGGLE THE LED STATE : 
    Serial.print("T1="); Serial.println(t1);
    Serial.print("DELAY="); Serial.println(d1);
    Serial.print("T2="); Serial.println(t2);
    Serial.print("WAIT="); Serial.println(d2);
    digitalWrite(LED_PIN,HIGH); // LED on
    digitalWrite(RELAY_PIN,HIGH); // Relay on
    delay(t1);
    digitalWrite(LED_PIN,LOW); // LED OFF
    digitalWrite(RELAY_PIN,LOW); // Relay OFF
    delay(d1);
    digitalWrite(LED_PIN,HIGH); // LED on+
    digitalWrite(RELAY_PIN,HIGH); // Relay on
    delay(t2);
    digitalWrite(LED_PIN,LOW); // LED OFF
    digitalWrite(RELAY_PIN,LOW); // Relay OFF
    u8x8.drawString(0, 4, "   Done ");
    delay(d2);
    Serial.println("End delay d2");
    updateDisplay = true; //force to display again the screen even if state/editing did not changed
}



int16_t minMax(int16_t val, int16_t min, int16_t max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

void checkRotaryEncoder(){
  static int pos = 0;

  encoder->tick(); // just call tick() to check the state.

  int newPos = encoder->getPosition();
  int dir = (int)(encoder->getDirection());
  if (pos != newPos) {
    int16_t posDif = newPos - pos;
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println(dir);
    Serial.print(" dir2:");
    Serial.println(dir);
    
    pos = newPos;
// when we rotate the button, 
//      If editing is false, then State change from Blank, Set Pre, Set Wait, Set Weld, Save
//      If editing is true, then we change pre, wait, weld value (within the limits and with a step of 10)
    if (editing) {
      switch (state){
        case PRE:
          t1 = minMax(t1 + (posDif *10),10,990);
          updateDisplay = true;
          break;
        case WAIT:
          d1 = minMax(d1 + (posDif *10),10,990);
          updateDisplay = true;
          break;
        case WELD:
          t2 = minMax(t2 + (posDif *10),10,990);
          updateDisplay = true;
          break;
      }
    } else { // not in editing mode
      state = state + dir;
      if (state < BLANK) state = BLANK;
      if (state > SAVE) state = SAVE;
    }
  } 
}

void checkRotaryButton(){
// UPDATE THE BUTTON
  // YOU MUST CALL THIS EVERY LOOP
  rotaryButton.update();
  // <Button>.pressed() RETURNS true IF THE STATE CHANGED
  // AND THE CURRENT STATE MATCHES <Button>.setPressedState(<HIGH or LOW>);
  // WHICH IS LOW IN THIS EXAMPLE AS SET WITH button.setPressedState(LOW); IN setup()
  if ( rotaryButton.pressed() ) {
    // When we press the button
    //      If state = blank, no action
    //      if state = Save, save the current values pre, wait, weld and then set state on blank
    //      if state = pre,wait,weld, change the flag editing
    if ((state == PRE) or (state == WAIT) or (state == WELD)){
      if (editing) {
        state = BLANK; //Force menu to go back to blank after editing
      }
      editing = !editing;
    
    } else if (state == SAVE) {
      saveConfig(); // save in eeprom and go back to Blank state
    }    
  } 
}

void setupLcd(void) {
  u8x8.begin();
  u8x8.clear(); // not sure it is required
}

void loop() {
  recvWithEndMarker();
  processCmd(); // process command received via uart (T1=xxx, D1=xxx, T2=xxx, D2 =xxx)

  //Detect and manage rotation of the encoder 
  checkRotaryEncoder();
  // detect and manage rotary push button
  checkRotaryButton();
  // update the screen (if there are changes in state or editing flag)
  fillScreen();

  // UPDATE THE BUTTON
  // YOU MUST CALL THIS EVERY LOOP
  button.update();
  // <Button>.pressed() RETURNS true IF THE STATE CHANGED
  // AND THE CURRENT STATE MATCHES <Button>.setPressedState(<HIGH or LOW>);
  // WHICH IS LOW IN THIS EXAMPLE AS SET WITH button.setPressedState(LOW); IN setup()
  if ( button.pressed() ) {
    performWelding();    
  }
}