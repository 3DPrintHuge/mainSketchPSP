#include <eRCaGuy_Timer2_Counter.h>

const int ledPin = 2;

const int pulPinPlat    = 5;
const int dirPinPlat    = 4;
const int enPinPlat     = 3;


const int pulPinVert  = 9;
const int dirPinVert  = 8;
const int enPinVert   = 7;

const int endPin = A0;

int stepDelay = 100;

int spindlePitch= 5; // [mm]

// platform motor parameters
float platGain              = 1;
signed long dplatPPS        = 1600*platGain; // desired platform pulse frequence [pulses/second]
unsigned long platPeriod    = 625;  // desired platform pulse period  [microseconds]
unsigned long platPPR       = 1600; // pulses per revolution on platform motor driver
unsigned long platPos       = 0;    // platform motor position [steps]
unsigned long setPlatPos    = 0;    // desired platform motor position [steps]

// vertical motor parameters
float vertGain              = 1;
unsigned long dvertPPS      = 1600*vertGain; // desired vertical pulse frequency [pulses/second]
unsigned long vertPeriod    = 625;  // desired verticval period [microseconds]
unsigned long vertPPR       = 1600; // pulses per revolution on vertical motor driver
unsigned long vertPos       = 0;    // vertical motor position [steps]
unsigned long vertPosmm     = 0;
unsigned long setVertPos    = 0;    // desired vertical motor position [steps]

// platform motor variables
unsigned long platRampPeriod  = 0;  // platform pulse period during ramping [microseconds]
signed long cplatPPS          = 1;  // curreny platform pulse frequency [pulses/second]
signed long platFreqStep      = 0;  // incemental value for platform pulse frequency while ramping [pulses/second]
unsigned long cTimePlatform   = 0;  // current time
unsigned long pTimePlatform   = 0;  // previous time

// vertical motor variables
unsigned long vertRampPeriod  = 0;  // platform pulse period during ramping [microseconds]
unsigned long cvertPPS        = 1;  // curreny platform pulse frequency [pulses/second]
unsigned long vertFreqStep    = 0;  // incemental value for platform pulse frequency while ramping [pulses/second]
unsigned long cTimeVert       = 0;  // current time
unsigned long pTimeVert       = 0;  // previous time

// global timer variables
unsigned long cMillis   = 0;  // current time
unsigned long pMillis   = 0;  // previous time

// ramp variables
bool rampBool  = true;
unsigned long rampTime  = 0.5 * 1000; //time in milliseconds
int rampInterval        = 100; // time between ramp steps [ms]
int rampi               = 1;  // ramp counter
signed int rampSteps    = 50; //amount of steps in ramp

// switch parameters
int endRead     = 0;  //read value from endswitch
bool endBool    = false;  //read value of endswitch converted to boolean

// random booleans
bool z  = true;

// Command interpreter variables
String inputString    = "";
String valString      = "";
String commandString  = "";
String returnEntry    = "";
unsigned long value   = 0;
bool stringEnd        = false;
bool cmdBool          = false;

// Command booleans
bool disableBool  = true;   // motors are disabled [true] motors are enabled [false]
bool slowBool     = false;  // shows if slowAll command is active 
bool runspdBool   = false;  // shows if runspd command is active
bool platBool     = true;
bool vertBool     = true;
bool incPlatBool  = false;  // shows if incPlat command is active
bool incVertBool  = false;  // shows if incVert command is active
bool platDirBool  = false;  // shows if platDirBool command is active
bool vertDirBool  = false;  // shows if vertDirBool command is active
bool initBool     = false;  // shows if init command is active

int progCount = 1;
unsigned long progStartTime= 0;
unsigned long progEndTime =0;
float diff = 0;
float diffMean=0;

unsigned long cMicros= 0 ;

enum Commands { PlatPPS, PlatDir, PlatPPR, MovePlat, IncPlat,
                VertPPS, VertDir, VertPPR, MoveVert, IncVert,
                SetRamp, Initialize, RunSpd, SlowAll, StopAll,
                Default};

enum Mode {DisableAll, EnableAll, Homing, Speed, StepPlatform, StepVertical };

Mode mode = DisableAll;
Commands inputCommand = Default;


void setup() {
  //put your setup code here, to run once:

  pinMode (ledPin, OUTPUT);
  pinMode(endPin, INPUT );

  pinMode (pulPinPlat, OUTPUT);
  pinMode (enPinPlat, OUTPUT);
  pinMode (dirPinPlat, OUTPUT);

  pinMode (pulPinVert, OUTPUT);
  pinMode (enPinVert, OUTPUT);
  pinMode (dirPinVert, OUTPUT);

  digitalWrite(pulPinPlat, HIGH);
  digitalWrite(dirPinPlat, HIGH);
  digitalWrite(enPinPlat, HIGH);

  digitalWrite(pulPinVert, HIGH);
  digitalWrite(dirPinVert, HIGH);
  digitalWrite(enPinVert, HIGH);

  timer2.setup();
  
  platPos       = 0;

  Serial.begin(2000000);

}

void loop() {
  
  // put your main code here, to run repeatedly:

  switch (mode) {
    
    case DisableAll:

      digitalWrite(enPinPlat, LOW);
      digitalWrite(enPinVert, LOW);
      z = true;
      break;
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ //
    case EnableAll:

      
      if (z) {
        digitalWrite(enPinPlat, HIGH);
        digitalWrite(enPinVert, HIGH);
             
        Serial.println("delay");
        delay(2500);
        z = false;
      }
      break;
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ //
    case Homing:
        homing();
      break;
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ //
    case Speed:
          if (z) {
        digitalWrite(enPinPlat, HIGH);
        digitalWrite(enPinVert, HIGH);
             
        Serial.println("delay");
        delay(2500);
        z = false;
      }
      
    if(runspdBool){runspd();}
    else if (slowBool){slowAll();}
    
      break;
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ //
    case StepPlatform:
        stepPlatform();
        incPlatBool = false;
      break;
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ //
    case StepVertical:
        stepVertical();
        incVertBool = false;
        break;
    default:
    break;
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ //
  }


}

void serialEvent() {

  while (Serial.available()) {

    char inChar = (char)Serial.read();
    returnEntry += inChar;

    //build input to command
    if (cmdBool == false && inChar != ';') {
      inputString += inChar;
    }

    //spacer detected
    if (inChar == ';') {
      cmdBool = true;
    }

    //build input value
    if (cmdBool == true && inChar != ';') {
      valString += inChar;
    }

    //end of input
    if (inChar == '~') {
      stringEnd = true;
      cmdBool = false;

      //activate decode message function
      decodeMessage();
    }

  }

}

void decodeMessage() {

  // ~~~~~~~~~~~~~~~~~ get input value ~~~~~~~~~~~~~~~~~ //
  if (inputString != "" && stringEnd == true) {

    value = valString.toInt();  //change from string to integer
    commandString = inputString;

    //Serial.print("inputString: ");
    //Serial.print(inputString);
    //Serial.print("\n");
    
    //Serial.print("value: ");
    //Serial.print(value);
    //Serial.print("\n");

    //reset parameters
    valString = "";
    inputString = "";
    stringEnd = false;
  }

  // + - + - + - + - + - + COMMAND INTERPRETER + - + - + - + - + - + //

  //  LIST OF COMMANDS
  //  -platPPS; pulses per second~
  //  -platDir; direction (0/1)~
  //  -platPPR; pulses per revolution~
  //  -movePlat; distance in steps~
  //  -incPlat; no value (0)~  
  //  -vertPPS; pulses per seconde~
  //  -vertDir; direction (0/1)~
  //  -vertPPR; pulses per revolution~
  //  -moveVert; distance in steps~
  //  -incVert; no value (0)~  
  //  -setRamp; ramp time in milliseconds~
  //  -init; no value (0)~
  //  -slowAll; no value (0)~
  //  -stopAll; no value (0)~
  //  -runspd; no value (0)~
  //  -command; value~

  //prototype
//    else if (commandString == "command") {
//      doStuff
//  
//      commandString = "";
//    }

//  --- VALUE RELATED COMMANDS ---  //

  //set platform pulses per second
  if (commandString == "platPPS") {
    dplatPPS= value;  //set platform pulses per second
    platPeriod = (1000000 / dplatPPS); //setplatform period in microseconds

    Serial.print("platPPS: ");
    Serial.print( dplatPPS);
    Serial.print("\n\r");
    Serial.print("platPeriod: ");
    Serial.print( platPeriod);
    Serial.print("\n\r");

    commandString = ""; //clear string
  }

  //set platform direction
  else if (commandString == "platDir") {

    //set direction parameter for platform motor
    if (value > 0) {
      platDirBool = true;
    } else {
      platDirBool = false;
    }

    setDir(); //activate direction change function

    commandString = ""; //clear string
  }

  //set platform pulses per revolution
  else if (commandString == "platPPR") {
    platPPR = value; //set platform pulses per period

    commandString = ""; //clear string
  }

  //rotate platform to position
  else if (commandString == "movePlat") {
    setPlatPos = value; //set desired platform position
    disableBool = false;
    gotoPlat(); //activate move to position function

    commandString = ""; //clear string
  }

  //set vertical ppulses per second
  else if (commandString == "vertPPS") {
   dvertPPS= value;  //set vertical pulses per second
       vertPeriod = (1000000 / dvertPPS); //setplatform period in microseconds
    Serial.print("vertPPS: ");
    Serial.print( dvertPPS);
    Serial.print("\n\r");

    commandString = ""; //clear string
  }

  //set vertical direction
  else if (commandString == "vertDir") {

    //set direction parameter for vertical motor
    if (value > 0) {
      vertDirBool = true;
    } else {
      vertDirBool = false;
    }

    setDir(); //activate direction change function

    commandString = ""; //clear string
  }

  //set vertical pulsesper revolution
  else if (commandString == "vertPPR") {
    vertPPR = value; //set vertical pulses per revolution

    commandString = ""; //clear string
  }

  //move vertically to position
  else if (commandString == "movePlat") {
    setPlatPos = value; //set desired platform position
    disableBool = false;
    gotoPlat(); //activate move to position function

    commandString = ""; //clear string
  }

  //set ramptime in [ms]
  else if (commandString == "setRamp") {
    rampTime = value; //set ramp time
    Serial.print("rampTime [ms]: ");
    Serial.print(rampTime);
    Serial.print("\n\r");

    commandString = ""; //clear string
  }
  
  else if (commandString == "setInterval") {
      rampInterval = value;
  
      commandString = "";
    }

  else if (commandString == "getPlatPos") {
      Serial.println(platPos);
  
      commandString = "";
    }
  
  // global calculations //

  
  //Serial.print("rampi: ");Serial.print(rampi);Serial.print("\n\r");
  //Serial.print("rampSteps: ");Serial.print(rampSteps);Serial.print("\n\r");

//  --- VALUE INDEPENDANT COMMANDS ---  //

    if (commandString == "init") {
      initBool =true;
      disableBool = false;
      mode = Homing;
  
      commandString = "";
    }
 
  //slow all motors
  else if (commandString == "slowAll") {
    slowBool = true;
    runspdBool = false;
    
    //rampi=rampSteps;

    commandString = "";
  }

  //emergency stop all motors
  else if (commandString == "stopAll") {
    disableBool = true;
    slowBool = false;
    runspdBool = false;
    mode= DisableAll;
    Serial.println("disabled");

    commandString = ""; //clear string
  }

  //run platform based on speed
  else if (commandString == "runspd") {

    if(!runspdBool) {
      //reset parameters
      cplatPPS = 1;
      cvertPPS = 1;
    }

    // if both motors are disabled, enable them
    if(!platBool && !vertBool){     
      platBool= true;
      vertBool= true;
    }

    
    runspdBool = true;
    slowBool= false;
    disableBool = false;
    mode= Speed;

    
    //rampi=0;

    commandString = ""; //clear string
  }

  //send a pulse to platform motor
  else if (commandString == "incPlat") {
    incPlatBool = true;
    disableBool = false;
    mode= StepPlatform;

    commandString = ""; //clear string
  }

  //send a pulse to vertical motor
  else if (commandString == "incVert") {
    incVertBool = true;
    disableBool = false;
    mode = StepVertical;

    commandString = ""; //clear string
  }

  // ENABLE/DISABLE MOTORS
  else if (commandString == "enAll") {
    platBool= true;
    vertBool= true;
    digitalWrite (enPinPlat, HIGH);
    digitalWrite (enPinVert, HIGH);

    if(!runspdBool) {
   //reset parameters
    cplatPPS = 1;
   cvertPPS = 1;
    }
    
    commandString = "";
  }

  else if (commandString == "enPlat") {
    platBool= true;
    digitalWrite (enPinPlat, HIGH);

    if(!runspdBool) {
      //reset parameters
      cplatPPS = 1;
    }

    commandString = "";
  }

  else if (commandString == "enVert") {
    vertBool= true;
    digitalWrite (enPinVert, HIGH);
    if(!runspdBool) {
      //reset parameters
      cvertPPS = 1;
    }

    commandString = "";
  }

  else if (commandString == "disPlat") {
    platBool= false;
    digitalWrite (enPinPlat, LOW);

    commandString = "";
  }

  else if (commandString == "disVert") {
    vertBool= false;
    digitalWrite (enPinVert, LOW);

    commandString = "";
  }

  // + - + - + - + end of interpreter + - + - + - +  //

  calcRampi();
  
  if (platPeriod > vertPeriod){stepDelay= vertPeriod/2;}
  else {stepDelay = platPeriod/2;}

  if(stepDelay<10){
    stepDelay=10;
  }

  rampBool = true;
}

void setDir() {
  
      digitalWrite(enPinPlat, LOW);
      digitalWrite(enPinVert, LOW);
  if (platDirBool) {
    digitalWrite(dirPinPlat, HIGH);

  } else {
    digitalWrite(dirPinPlat, LOW);
  }

  if (vertDirBool) {
    digitalWrite(dirPinVert, HIGH);
  }
  else {
    digitalWrite(dirPinVert, LOW);
  }
        digitalWrite(enPinPlat, HIGH);
      digitalWrite(enPinVert, HIGH);

}

void stepPlatform() {

  //toggle pulpin for one step
  PORTD = PORTD^B00100000;
  delayMicroseconds(stepDelay);
  PORTD = PORTD|B00100000;

  if (platDirBool){    
    if (platPos >= platPPR){ platPos=0; }
    platPos++; 
    }
    
  else{ 
    if (platPos == 0) {platPos= platPPR;}
    platPos--;
    }   
  
}

void stepVertical() {

  //toggle pulpin for one step
  PORTB = PORTB^B00000010;
  delayMicroseconds(stepDelay);
  PORTB = PORTB|B00000010;

  if (vertDirBool){ vertPos++;}
  else{ vertPos--;};
  vertPosmm= (vertPos/vertPPR)*spindlePitch;
  
}

void runspd(){

  cMicros = micros();
  cMillis = millis();
  progStartTime= micros();

  platRampPeriod = (1000000 / cplatPPS); // pulse period of platform motor
  vertRampPeriod = (1000000 / cvertPPS);  //pulse period of vertical motor
  // ^ period will increase with respect to rampi and will eventually reach value set by the user ^

      //  ~ ~ ~ ~ PLATFORM STEP ~ ~ ~ ~   //
      if ( (cMicros - pTimePlatform >= platRampPeriod) && platBool ) {
        PORTD = PORTD^B00100000;      //SWITCH POLARITY OF PIN 5, THIS IS FASTER THAN DIGITALWRITE()
        delayMicroseconds(stepDelay);
        PORTD = PORTD|B00100000;      //SWITCH POLARITY OF PIN 5, THIS IS FASTER THAN DIGITALWRITE()
        pTimePlatform = cMicros;
    }

    //  ~ ~ ~ ~ VERTICAL STEP ~ ~ ~ ~   //
    if ( (cMicros - pTimeVert >= vertRampPeriod) && vertBool ) {
      PORTB = PORTB^B00000010;      //SWITCH POLARITY OF PIN 9, THIS IS FASTER THAN DIGITALWRITE()
      delayMicroseconds(stepDelay);
      PORTB = PORTB|B00000010;      //SWITCH POLARITY OF PIN 9, THIS IS FASTER THAN DIGITALWRITE()
      pTimeVert = cMicros;
    }

    //  ~ ~ ~ ~ RAMP STEP ~ ~ ~ ~   //
    if ( (cMillis - pMillis >= rampInterval) && rampBool) {

      if (rampi < rampSteps) {
        cplatPPS = cplatPPS + platFreqStep;
        cvertPPS = cvertPPS + vertFreqStep;
        rampi++;
      }
      
      if (rampi == rampSteps) {
        cplatPPS = dplatPPS;
        cvertPPS = dvertPPS;
        Serial.println("max speed");
        rampBool = false;
      }

      pMillis = cMillis; // update previous time

    }
    
}

void slowAll(){
    cMicros = micros();
    cMillis = millis();

   platRampPeriod = (1000000 / cplatPPS); // pulse period of platform motor
  vertRampPeriod = (1000000 / cvertPPS);  //pulse period of vertical motor
  // ^ period will increase with respect to rampi and will eventually reach value set by the user ^
  
      if ( (cMicros - pTimePlatform >= platRampPeriod) && platBool ) {
      PORTD = PORTD^B00100000;
      delayMicroseconds(stepDelay);
      PORTD = PORTD|B00100000;
      pTimePlatform = cMicros;

    }

    if ( (cMicros - pTimeVert >= vertRampPeriod) && vertBool ) {
      PORTB = PORTB^B00000010;
      delayMicroseconds(stepDelay);
      PORTB = PORTB|B00000010;
      pTimeVert = cMicros;

    }

    if (cMillis - pMillis >= rampInterval && rampBool) {

      if (rampi > 1) {
        cplatPPS = cplatPPS - platFreqStep;
        cvertPPS = cvertPPS - vertFreqStep;
        rampi--;
      }

      if (rampi == 1) {
        cplatPPS = 1;
        cvertPPS = 1;
        rampBool = false;
        slowBool = false;
        disableBool = true;
        mode= DisableAll;
        Serial.println("stopped");    
      }
      
      pMillis = cMillis;

    }


  
}

// change plat to vert
void homing(){
  platDirBool = false;
  endRead = analogRead(endPin);
  if(endRead>512){endBool = true;} else {endBool =false;};
  setDir();
  
  while(!endBool){
    endRead = analogRead(endPin);
    if(endRead>512){endBool = true;} else {endBool =false;};
    stepPlatform();
    delay(1);
    //Serial.println(endBool);
  }
  platDirBool = true;
  platPos = 0;
  
  initBool =false;
  
}

void gotoPlat() {
  if (!disableBool) {
    //enable platform motor
    digitalWrite(enPinPlat, HIGH);
    delay(2500);

    //move platform until position is reached
    while (platPos != setPlatPos) {
      //Serial.println(platPos);
      stepPlatform();
      delayMicroseconds(platPeriod);
      
      //if (setPlatPos == platPPR){setPlatPos=0;};
      Serial.println(setPlatPos);
    }

    //disable platform motor
    digitalWrite(enPinPlat, LOW);
  }
}

int calcRampi() {
  
  if (runspdBool){
    rampSteps = rampSteps;
    platFreqStep = (dplatPPS-cplatPPS)/rampSteps;
    rampi=1;
    Serial.println(platFreqStep);
    
  }
  else if(slowBool){
     rampSteps = rampSteps;
     platFreqStep = cplatPPS/rampSteps;
  }
  else {
    rampSteps =  (rampTime / rampInterval);
      platFreqStep = dplatPPS/ rampSteps;
      vertFreqStep = dvertPPS/ rampSteps;  
    
  }
  
}

