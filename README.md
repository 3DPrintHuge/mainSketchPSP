# Polymer Science Park Testopstelling
   
  Author(s):
  Sven Dicker - sdsdsven@gmail.com - 0639842173
  
  This program is part of a test setup for large scale 3D printing. The system consists of 3 different controlling components: a pc, an arduino and a labjack. The pc is the master of the wholesetup, the arduino and labjack are slaves.
     
  The arduino is responsible for controlling the motor drivers and reading the endswitches. The parameters and methods/modes of controlling the motor drivers are based on commands received over serial communication.   There are two types of commands parameter: parameter commands and excecution commands. Parameter commands have a relevant parameter whereas excecution commands do not. However a parameter has to be given to every send command, in the case of an excecution command this parameter will usually be 0 but the value is redundant.   
  

## The program
Every arduino program has two main functions: `setup()` and `loop()`. There is also a third function which is usually hidden but used in this application `serialEvent()`. `serialEvent()` is triggered after every `loop()` itteration.

### `serialEvent()`
The function `serialEvent()` looks for a stream of data over USB. If data is available the data will be extracted from the stream and is inpected character by character. Flags are added to the command string to tell the difference between the command and the parameter. The command and parameter are build as a global strings `inputString` and `valString` which can be interpreted by the function `decodeMessage()`. When an end flag is detected the function `decodeMessage()` is called.

### `decodeMessage()`
  The function `decodeMessage` is used as command interpreter, `inputString` is renamed to `commandString` and `valString` is casted to a double type and is renamed to `value`. The interpreter looks at `commandString` and compares it a set of known commands. If `commandString` equals a known command certain variables will be set accordingly. 

### `loop()`
The function `loop()` is run continuously and consists mainly of a switch statement which control the mode. There are 7 modes: 
  
  - `DisableAll` - disables all motors, no current will be applied the windings so the motrs are free to rotatethis is the default mode.
  
  - `Break` - stops pulsing the motor drivers but keeps the motors enable. This mode is used after jog mode because of the delay that occurs in the drivers when the motors switch from disabled to enabled.
  
  - `Homing` - sets the vertical direction upward and then enables the vertical motor. When the top end witch is activated the motors stop and move downward until the switch is deactivated. Then the DisableAll mode is activated.
  
  - `Speed` - motors are driven based on a set pulse period for each motor
  
  - `Print` - platform moter is driven based on speed, vertical motor is driven so that the platform will drop a certain distance for every rotation.
  
  - `JogPlatform`/`JogVertical` - will drive a certain motor based on the set pulse period, pretty musch the same as Speed only for one specified motor.
  
