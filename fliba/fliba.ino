#include <XInput.h>

#ifndef clear_bit
#define clear_bit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef set_bit
#define set_bit(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

const int ringBufferSize = 20;
int xBufferIndex = 0;
int yBufferIndex = 0;
int xBumpArray[ringBufferSize];
int yBumpArray[ringBufferSize];
int deadzone = 12;
int sensitivity=50;
bool replicateTriggersToButtons = false;
bool disableBump=false;
bool settingsActive = false;

const bool valuesFromDisplay = true;
const int leftTriggerInput = 10;
const int rightTriggerInput = 3;
const unsigned long  uiUpdateInterval = 100000;

// Button Setup
const int ButtonId2ButtonCode[] = {
  BUTTON_BACK,
  BUTTON_START,
  BUTTON_A,
  BUTTON_B,
  BUTTON_X,
  BUTTON_Y,
  DPAD_UP,
  DPAD_DOWN,
  DPAD_LEFT,
  DPAD_RIGHT,
  BUTTON_LOGO
};

typedef struct Input2ButtonMap {
  int8_t digitalInput;
  int button;
  bool lastValue;
} I2B;

const int numMappings = 6;
Input2ButtonMap mapping[] = {{9, BUTTON_LB, 0}, {4, BUTTON_RB, 0}, {2, BUTTON_A, 0}, {6, BUTTON_Y, 0}, {8, BUTTON_X, 0}, {11, BUTTON_B, 0} };

void setup() {
  clearArray(xBumpArray, ringBufferSize);
  clearArray(yBumpArray, ringBufferSize);
  ui_setupSerial();
  setupAnalogInputs();
  setupXInput();
  setupPins();
  Serial.println("Ready");
}

void loop() {
  /*if (digitalRead(SafetyPin) == LOW) {
    return;
  }*/
  static unsigned long sinceUiUpdate = 0;
  static unsigned long avgTime = 0;
  unsigned long startTime = micros();  // Get timestamp for comparison

  // DPad
  //XInput.setDpad(dpadPosition == 0, dpadPosition == 1, dpadPosition == 2, dpadPosition == 3)

  updateDigitals();
  updateAnalogs();
  updateFromDisplay();

  if(replicateTriggersToButtons)
  {
    updateButtonsFromTriggers();
  }

  // Send values to PC
  XInput.send();

  // calculate loop time 
  unsigned long loopTime = micros()-startTime;
  avgTime = (loopTime + avgTime) / 2;
  sinceUiUpdate = sinceUiUpdate + loopTime;
  if(settingsActive && sinceUiUpdate > uiUpdateInterval)
  {
    
    sinceUiUpdate = 0;
    updateSettingsUI(avgTime);
  }
}

void updateSettingsUI(unsigned long looptime)
{
  ui_setValue("ct.val", String(looptime));
  ui_setValue("sens.val", String(sensitivity));
  ui_setValue("sensslider.val", String(sensitivity));
  ui_setValue("dz.val", String(deadzone));
  ui_setValue("dzslider.val", String(deadzone));
}

void updateButtonsFromTriggers()
{
  bool bright = XInput.getButton(BUTTON_RB);
  bool bleft = XInput.getButton(BUTTON_LB);
  if(bright)
    XInput.setTrigger(TRIGGER_RIGHT, bright );
  if(bleft)
    XInput.setTrigger(TRIGGER_LEFT, bleft);
}

void updateDigitals()
{
  for (int x = 0; x<numMappings; x++)
  {
     bool value = !digitalRead(mapping[x].digitalInput);
     if(value != mapping[x].lastValue) XInput.setButton(mapping[x].button, value);
     mapping[x].lastValue = value;
  }
}

void updateAnalogs()
{
  static int minA1Analog = -1;
  static int maxA1Analog = -1;
  static int minA0Analog = -1;
  static int maxA0Analog = -1;
  
  int leftTrigger = !digitalRead(leftTriggerInput);
  int rightTrigger =  !digitalRead(rightTriggerInput);
  if(XInput.getTrigger(TRIGGER_LEFT) != leftTrigger )
    XInput.setTrigger(TRIGGER_LEFT, leftTrigger);
  if(XInput.getTrigger(TRIGGER_RIGHT) != rightTrigger )
    XInput.setTrigger(TRIGGER_RIGHT, rightTrigger);

  if(disableBump)
    return;

  static int32_t lastJ1x = 0;
  static int32_t lastJ1y = 0;
  static int32_t lastJ2y = 0;

  int a1= analogRead(A1);
  int a0= analogRead(A0);
  int32_t j1x = getJoyValueFromAnalog( a1, xBumpArray, ringBufferSize, xBufferIndex);
  int32_t j1y = getJoyValueFromAnalog( a0, yBumpArray, ringBufferSize, yBufferIndex);
  if(lastJ1x != j1x || lastJ1y != j1y) XInput.setJoystick(JOY_LEFT, j1x, j1y);
  lastJ1x = j1x;
  lastJ1y = j1y;

   //int32_t j2y = analogRead(A2) >> 2; // plunger in display
  //if(lastJ2y != j2y) XInput.setJoystick(JOY_RIGHT, 0, j2y);
  //lastJ2y = j2y;
}

void clearArray(int intArray[], const int arraySize)
{
  for (int x = 0; x < arraySize ; x++)
  {
    intArray[x] = 0; 
  }
}

float arrayAverage(int intArray[], const int arraySize)
{
  int total = 0;
  for (int x = 0; x < arraySize ; x++)
  {
    total += intArray[x]; 
  }
  return float(total) / arraySize;
}

int getJoyValueFromAnalog(int analogValue, int intArray[], const int arraySize, int &index)
{
  intArray[index] = analogValue;
  index++;
  if (index >= arraySize) index = 0;
  float average = arrayAverage(intArray, arraySize);
  //float bumpValue = analogValue - average;
  int ret = map(analogValue, average-sensitivity, average+sensitivity,-127,128);
  if(abs(ret) > deadzone)
  {
    //Serial.print("value;");
    //Serial.print(String(analogValue));
    //Serial.print(",ret:");
    //Serial.print(String(ret));
    //Serial.print(",avg;");
    //Serial.println(String(arrayAverage(intArray, arraySize)));
    if(ret<-127) ret=-127;
    if(ret>128) ret=128;
    return ret;
  }
  return 0;
}

void ui_setValue(const String& valueName, const String& value)
{
  Serial1.print(valueName);
  Serial1.print("=");
  Serial1.print(value);
  Serial1.print("\xFF\xFF\xFF");
}
                             
void ui_setupSerial()
{
  Serial.begin(9600); // serial monitor
  Serial1.begin(9600);
  delay(500);
  ui_setValue("baud", "115200");
  Serial1.end();
  Serial1.begin(115200);

}

void setupAnalogInputs()
{
    //pinMode(SafetyPin, INPUT_PULLUP);
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
  set_bit(ADCSRA,ADPS2) ;
  clear_bit(ADCSRA,ADPS1) ;
  clear_bit(ADCSRA,ADPS0) ;  
}

void setupXInput()
{
  XInput.setAutoSend(false);  
  XInput.begin();
  XInput.setTriggerRange(0, 1);
  XInput.setJoystickRange(-127, 127);
}

void setupPins()
{
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);
  pinMode(A1, INPUT);
  digitalWrite(A1, HIGH);

  pinMode(leftTriggerInput, INPUT);
  digitalWrite(leftTriggerInput, HIGH);
  pinMode(rightTriggerInput, INPUT);
  digitalWrite(rightTriggerInput, HIGH);
  for (int x = 0; x<numMappings; x++)
  {
    pinMode(mapping[x].digitalInput, INPUT);
    digitalWrite(mapping[x].digitalInput, HIGH);
  }
}

void readUntilEndOfCommand(){
  int numEndChars = 0;
  while(numEndChars!=3)
  {
    if(Serial1.read() == 0xff)
      numEndChars++;
    else
      numEndChars = 0;
  }
}

void plungerStart(int serialCommand[])
{
}

void plungerStop(int serialCommand[])
{
  XInput.setJoystick(JOY_RIGHT, 0, 0);
}

void plungerMoving(int serialCommand[])
{
  int dir = serialCommand[1];
  int val = serialCommand[2];
  XInput.setJoystick(JOY_RIGHT, 0, dir ? val : 0-val);
}

void handleSettingsPage(int serialCommand[])
{
  settingsActive  = serialCommand[1]>0;
}

void handleDisableBump(int serialCommand[])
{
  disableBump  = serialCommand[1]==0;
}

void handleSupport4Flippers(int serialCommand[])
{
  replicateTriggersToButtons  = serialCommand[1]==0;
}

void handleSetSensitivity(int serialCommand[])
{
  sensitivity = serialCommand[1];
}

void handleSetDeadzone(int serialCommand[])
{
  deadzone = serialCommand[1];
}

void handleTouchPressed(int serialCommand[])
{
  int code = serialCommand[1];
  
  switch(code)
  {
    case 0xa0:
      XInput.setTrigger(TRIGGER_LEFT,  0 );
      break;
    case 0xa1:
      XInput.setTrigger(TRIGGER_RIGHT,  0 );
      break;
    default:
      XInput.setButton(ButtonId2ButtonCode[code], 1);
      break;
  }
}

void handleTouchReleased(int serialCommand[])
{
  int code = serialCommand[1];
  switch(code)
  {
    case 0xa0:
      XInput.setTrigger(TRIGGER_LEFT,  1 );
      break;
    case 0xa1:
      XInput.setTrigger(TRIGGER_RIGHT,  1);
      break;
    default:
      XInput.setButton(ButtonId2ButtonCode[code], 0);
      break;
  }
}

void printCommand(int command[])
{ 
  int* commandItem=command;
  Serial.print("Command: ");
  do
  {
    Serial.print(String(*commandItem));
    Serial.print(" ");
  } while (*commandItem++ != 0xff);
  Serial.println("");
}

int serialCommand[10];

void updateFromDisplay()
{
  static int commandIndex = 0;
  if(!Serial1.available()) return;

  while(Serial1.available())
  {
    int code = Serial1.read();
    serialCommand[commandIndex++] = code;
    if(code!=0xff) // command not ready
      break;
  }
  
  if(serialCommand[commandIndex-1]!=0xff)
    return;
    
  //printCommand(serialCommand);
  commandIndex = 0;
  switch(serialCommand[0])
  {
    case 1: 
      plungerStart(serialCommand);
      break;
    case 2: 
      plungerMoving(serialCommand);
      break;
    case 3: 
      plungerStop(serialCommand);
      break;
    case 4: // touch pressed
      handleTouchPressed(serialCommand);
      break;
    case 5: // touch release
      handleTouchReleased(serialCommand);
      break;
    case 6: // settings page on/off
      handleSettingsPage(serialCommand);
      break;
    case 7: // enable/diable bumping
      handleDisableBump(serialCommand);
      break;
    case 8: // are rb same as right trigger and same for left side..
      handleSupport4Flippers(serialCommand);
      break;
    case 9: 
      handleSetSensitivity(serialCommand);
      break;
    case 10:
      handleSetDeadzone(serialCommand);
      break;
      
    default:
      Serial.println("unknown command;");
      //readUntilEndOfCommand(); 
      break;
  }
}
