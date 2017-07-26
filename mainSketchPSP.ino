#include <eRCaGuy_Timer2_Counter.h>

const int ledPin = 2;

const int pulPinPlat    = 5;
const int dirPinPlat    = 4;
const int enPinPlat     = 3;


const int pulPinVert  = 9;
const int dirPinVert  = 8;
const int enPinVert   = 7;

const int endPin = A0;

const int stepDelay = 100;

// platform motor parameters
float platGain              = 1.25;
signed long dplatPPS        = 1600*platGain; // desired platform pulse frequence [pulses/second]
unsigned long platPeriod    = 625;  // desired platform pulse period  [microseconds]
unsigned long platPPR       = 1600; // pulses per revolution on platform motor driver
unsigned long platPos       = 0;    // platform motor position [steps]
unsigned long setPlatPos    = 0;    // desired platform motor position [steps]

// vertical motor parameters
float vertGain              = 1.25;
unsigned long dvertPPS      = 1600*vertGain; // desired vertical pulse frequency [pulses/second]
unsigned long vertPeriod    = 625;  // desired verticval period [microseconds]
unsigned long vertPPR       = 1600; // pulses per revolution on vertical motor driver
unsigned long vertPos       = 0;    // vertical motor position [steps]
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
unsigned long rampTime  = 0.5 * 1000; //time in milliseconds
int rampInterval        = 10; // time between ramp steps [ms]
int rampi               = 1;  // ramp counter
signed int rampSteps    = 50; //amount of steps in ramp

// switch parameters
int endRead     = 0;  //read value from endswitch
bool endBool    = false;  //read value of endswitch converted to boolean

// random booleans
bool q  = true;
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

  if (disableBool) {
    digitalWrite(enPinPlat, LOW);
    digitalWrite(enPinVert, LOW);
    //digitalWrite(enPinPlat, HIGH);
    
    z = true;
  } else {
    digitalWrite(enPinPlat, HIGH);
    digitalWrite(enPinVert, HIGH);
    if (z) {
      Serial.println("delay");
      delay(2500);
      z = false;
    }

  }
  
  if (initBool){
    homing();
  }

  if (runspdBool ) {
    runspd();
  }

  if (slowBool) {
    slowAll();
  }

  if (incPlatBool) {
    stepPlatform();
    incPlatBool = false;
  }

  if (incVertBool) {
    stepVertical();
    incVertBool = false;
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

    Serial.print("inputString: ");
    Serial.print(inputString);
    Serial.print("\n");
    Serial.print("value: ");
    Serial.print(value);
    Serial.print("\n");

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
  //  if (commandString == "command") {
  //    doStuff
  //
  //    commandString = "";
  //  }

//  --- VALUE RELATED COMMANDS ---  //
  //set platform pulses per second
  if (commandString == "platPPS") {
    dplatPPS= value*platGain;  //set platform pulses per second
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
  if (commandString == "platDir") {

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
  if (commandString == "platPPR") {
    platPPR = value; //set platform pulses per period

    commandString = ""; //clear string
  }

  //rotate platform to position
  if (commandString == "movePlat") {
    setPlatPos = value; //set desired platform position
    disableBool = false;
    gotoPlat(); //activate move to position function

    commandString = ""; //clear string
  }

  //set vertical ppulses per second
  if (commandString == "vertPPS") {
   dvertPPS= value*vertGain;  //set vertical pulses per second
    Serial.print("vertPPS: ");
    Serial.print( dvertPPS);
    Serial.print("\n\r");

    commandString = ""; //clear string
  }

  //set vertical direction
  if (commandString == "vertDir") {

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
  if (commandString == "vertPPR") {
    vertPPR = value; //set vertical pulses per revolution

    commandString = ""; //clear string
  }

  //move vertically to position
  if (commandString == "movePlat") {
    setPlatPos = value; //set desired platform position
    disableBool = false;
    gotoPlat(); //activate move to position function

    commandString = ""; //clear string
  }

  //set ramptime in [ms]
  if (commandString == "setRamp") {
    rampTime = value; //set ramp time
    Serial.print("rampTime [ms]: ");
    Serial.print(rampTime);
    Serial.print("\n\r");

    commandString = ""; //clear string
  }


    if (commandString == "setInterval") {
      rampInterval = value;
  
      commandString = "";
    }
  
  // global calculations //

  
  Serial.print("rampi: ");Serial.print(rampi);Serial.print("\n\r");
  Serial.print("rampSteps: ");Serial.print(rampSteps);Serial.print("\n\r");

//  --- VALUE INDEPENDANT COMMANDS ---  //

    if (commandString == "init") {
      initBool =true;
      disableBool = false;
  
      commandString = "";
    }
 
  //slow all motors
  if (commandString == "slowAll") {
    slowBool = true;
    runspdBool = false;
    q= true;
    //rampi=rampSteps;

    commandString = "";
  }

  //emergency stop all motors
  if (commandString == "stopAll") {
    disableBool = true;
    slowBool = false;
    runspdBool = false;
    Serial.println("disabled");

    commandString = ""; //clear string
  }

  //run platform based on speed
  if (commandString == "runspd") {

    if(!runspdBool) {
      //reset parameters
      cplatPPS = 1;
      cvertPPS = 1;
    }

    
    runspdBool = true;
    slowBool= false;
    disableBool = false;
    q=true;

    
    //rampi=0;

    commandString = ""; //clear string
  }

  //send a pulse to platform motor
  if (commandString == "incPlat") {
    incPlatBool = true;
    disableBool = false;

    commandString = ""; //clear string
  }

  //send a pulse to vertical motor
  if (commandString == "incVert") {
    incVertBool = true;
    disableBool = false;

    commandString = ""; //clear string
  }

  // + - + - + - + end of interpreter + - + - + - +  //

  calcRampi();
  q = true;
}

void setDir() {

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

}

void stepPlatform() {

  //  Serial.println("stepplatform");

  //toggle pulpin for one step
//  digitalWrite(pulPinPlat, LOW);
//  delayMicroseconds(stepDelay);
//  digitalWrite(pulPinPlat, HIGH);

  PORTD = PORTD^B00100000;
  delayMicroseconds(stepDelay);
  PORTD = PORTD|B00100000;
  //Serial.println(platPos);
  //Serial.println(platPPR);
  if (platDirBool){ 
    
    if (platPos >= platPPR){ platPos=0; }
    platPos++;
    
    }
  else{ 
    
    if (platPos == 0) {platPos= platPPR;}
    platPos--;
    };   
  
}

void stepVertical() {

  //toggle pulpin for one step
//  digitalWrite(pulPinVert, LOW);
//  delayMicroseconds(stepDelay);
//  digitalWrite(pulPinVert, HIGH);

PORTB = PORTB^B00000010;
  delayMicroseconds(stepDelay);
  PORTB = PORTB|B00000010;

  if (vertDirBool){ vertPos++;}
  else{ vertPos--;};

  if (vertPos >= vertPPR){ vertPos=0;};
  if (vertPos == 0) {vertPos= (vertPPR-1);}
  

}

void runspd(){

  bool printsteptime= false;
  unsigned long per=0;


    //cTimePlatform = micros();
  //cTimeVert = micros();
  cMicros = micros();
  cMillis = millis();
    progStartTime= micros();
    unsigned long deltaPlat = cMicros - pTimePlatform;

   platRampPeriod = (1000000 / cplatPPS); // pulse period of platform motor
  vertRampPeriod = (1000000 / cvertPPS);  //pulse period of vertical motor
  // ^ period will increase with respect to rampi and will eventually reach value set by the user ^
  
      if ( deltaPlat >= (platRampPeriod) ) {
        printsteptime=true;
        per = cTimePlatform - pTimePlatform;

      PORTD = PORTD^B00100000;
      delayMicroseconds(stepDelay);
      PORTD = PORTD|B00100000;
      pTimePlatform = cMicros;

    }

    if (cMicros - pTimeVert >= (vertRampPeriod) ) {
      PORTB = PORTB^B00000010;
  delayMicroseconds(stepDelay);
  PORTB = PORTB|B00000010;
      //stepVertical();
      pTimeVert = cMicros;

    }

    if (cMillis - pMillis >= rampInterval && q) {

      if (rampi < rampSteps) {
        rampi++;

        cplatPPS = cplatPPS + platFreqStep;
        cvertPPS = cvertPPS + vertFreqStep;
        //Serial.println(cplatPPS);

       //Serial.print("rampi: ");Serial.print(rampi);Serial.print("\n\r");
        //Serial.print("rampSteps: ");Serial.print(rampSteps);Serial.print("\n\r");

      }
      if (rampi == rampSteps) {
        Serial.println("max speed");
        //Serial.println(cplatPPS);
        cplatPPS = dplatPPS;
        cvertPPS = dvertPPS;
        q = false;
      }

      pMillis = cMillis; // update previous time

    }

    
  progEndTime =micros();

  diff = progEndTime - progStartTime;

  diffMean= (diffMean+diff);

  //if(progCount == 100){Serial.println(diffMean/100); diffMean=0; progCount=0; }

//  if(printsteptime){
//    Serial.print("platperiod: ");Serial.print(platRampPeriod);Serial.print("\r\n");
//    Serial.print("dT: ");Serial.print(per);Serial.print("\r\n");}

  progCount++;

  
}

void slowAll(){
    cTimePlatform = micros();
  cTimeVert = micros();
  cMillis = millis();

   platRampPeriod = (1000000 / cplatPPS); // pulse period of platform motor
  vertRampPeriod = (1000000 / cvertPPS);  //pulse period of vertical motor
  // ^ period will increase with respect to rampi and will eventually reach value set by the user ^
  
    if (cTimePlatform - pTimePlatform >= (platRampPeriod) ) {

      stepPlatform();
      pTimePlatform = cTimePlatform;

    }

    if (cTimeVert - pTimeVert >= (vertRampPeriod) ) {
      stepVertical();
      pTimeVert = cTimeVert;

    }

    if (cMillis - pMillis >= rampInterval && q) {

      if (rampi > 1) {
        rampi--;
        //cplatPPS =  platPPS*((double)rampi/(double)rampSteps);
        //cvertPPS =  vertPPS*((double)rampi/rampSteps);

        //Serial.println(cplatPPS);

        cplatPPS = cplatPPS - platFreqStep;
        cvertPPS = cvertPPS - vertFreqStep;
        
        //Serial.print("rampi: ");Serial.print(rampi);Serial.print("\n\r");
        //Serial.print("rampSteps: ");Serial.print(rampSteps);Serial.print("\n\r");

      }

      if (rampi == 1) {
        Serial.println("stopped");
        cplatPPS = 1;
        cvertPPS = 1;
        q = false;
        
        slowBool = false;
        disableBool = true;
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


