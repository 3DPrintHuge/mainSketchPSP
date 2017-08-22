# Polymer Science Park Testopstelling
   
  Author(s):
  Sven Dicker - sdsdsven@gmail.com - 0639842173
  
  This program is part of a test setup for large scale 3D printing. The system consists of 3 different controlling components: a pc, an arduino and a labjack. The pc is the master of the wholesetup, the arduino and labjack are slaves.
     
  The arduino is responsible for controlling the motor drivers and reading the endswitches. The parameters and methods/modes of controlling the motor drivers are based on commands received over serial communication.   
  
  There are two types of commands: parameter commands and excecution commands. Parameter commands have a relevant parameter whereas excecution commands do not. However a parameter has to be given to every send command, in the case of an excecution command this parameter will usually be 0 but the value is redundant.   
  

## The program
Every arduino program has two main functions: `setup()` and `loop()`. There is also a third function which is usually hidden but used in this application `serialEvent()`. `serialEvent()` is triggered after every `loop()` itteration, and excecuted when data is available.

### `serialEvent()`
The function `serialEvent()` looks for a stream of data over USB. If data is available the new data will be extracted from the stream and is inpected character by character. Flags are added in the command string to tell the difference between the command and the parameter. Commands are closed by a semicolon `;` the whole string is closed by a tilde `~`. The command and parameter are build as the global strings `inputString` and `valString` which can be interpreted by the function `decodeMessage()`. When the end flag is detected the function `decodeMessage()` is called.

### `decodeMessage()`
  The function `decodeMessage` is used as command interpreter, `inputString` is renamed to `commandString` and `valString` is casted to a double type and is renamed to `value`. The interpreter looks at `commandString` and compares it a set of known commands. If `commandString` equals a known command certain variables will be set accordingly. 
 
 List of parameter commands:
 * platPPS; pulses per second~ 
 * platDir; direction (0/1)~
 * platPPR; pulses per revolution~
 * vertPPS; pulses per seconde~
 * vertDir; direction (0/1)~
 * vertPPR; pulses per revolution~
 * setRamp; ramp time in milliseconds~
 * getPlatPos; returns platform position in steps, no value (0)~
 * setLayer; layer height in millimeters~ 
 
 List of execution commands:
* init; no value (0)~
* slowAll; no value (0)~
* stopAll; no value (0)~
* runspd; no value (0)~
* runprint; no value(0)~
* jogPlat; no value (0)~
 * jogVert; no value (0)~
 * break; no value (0)~ 
 
 Disable/enable motors:
 * enAll; no value (0)~
 * enPlat; no value (0)~
 * enVert; no value (0)~
 * disPlat; no value (0)~
 * disVert; no value (0)~

----

### `loop()`
The function `loop()` is run continuously and consists of a switch statement which control the mode and two timers `cMicros` and `cMillis`. There are 7 modes/cases: 
  
  - `DisableAll` - disables all motors, no current will be applied the windings so the motrs are free to rotatethis is the default mode.
  
  - `Break` - stops pulsing the motor drivers but keeps the motors enable. This mode is used after jog mode because of the delay that occurs in the drivers when the motors switch from disabled to enabled.
  
  - `Homing` - sets the vertical direction upward and then enables the vertical motor. When the top end witch is activated the motors stop and move downward until the switch is deactivated. Then the DisableAll mode is activated.
  
  - `Speed` - motors are driven based on a set pulse period for each motor
  
  - `Print` - platform moter is driven based on speed, vertical motor is driven so that the platform will drop a certain distance for every rotation.
  
  - `JogPlatform`/`JogVertical` - will drive a certain motor based on the set pulse period, pretty musch the same as Speed only for one specified motor.
  
#### `DisableAll`
When the mode `DisableAll` is active all motors will be disabled by pulling the enable pins low.

#### `Break`
When the mode `Break` is active all motors will remain enabled but they wont be driven.

#### `Homing`
When the mode `Homing` is active the function `homing()` is called. In `homing()` the platform is moved upward until the top end switch is activated. When the top end switch is active the platform will stop and drop until the switch is inactive. When the switch is released the program will go into `DisableAll` mode.

#### `Speed`
When the mode `Speed` is active the functions `runspd()` or `slowAll()` will be called based on the booleans `runspdBool` and `slowBool`. Before either function is called the end switches are checked to see if either one is active if that is the case the program will switch to `DisableAll`.

#### `Print`
When the mode `Print` is active the functions `runprint()` or `slowAll()` will be called based on the booleans `printBool` and `slowBool`. Before either function is called the end switches are checked to see if either one is active if that is the case the program will switch to `DisableAll`.

#### `JogVertical`
When the mode `JogPlatform` is active the function `runspd()` is called. Before `runspd()` is called the end switches are checked to see if either one is active if that is the case the program will switch to `DisableAll`. When the command is received the vertical motor is automatically disabled.

#### `JogVertical`
When the mode `JogVertical` is active the function `runspd()` is called. Before `runspd()` is called the end switches are checked to see if either one is active if that is the case the program will switch to `DisableAll`. When the command is received the platform motor is automatically disabled.

----

### `runspd()`
The function `runspd()` controls the motor drivers based on speed. The drivers are pulsed with a certain period, because the two motors have different speeds they are pulsed seperately. When the elapsed time is greater than or equal to the pulse period the motor is pulsed with the function `stepPlatform` or `stepVertical`.

`runspd()` also incorporates a ramp function, the ramp function increments/decrements the pulse period every `rampInterval`. The ramp interval is a constant, when the elapsed time is greater than or equal to `rampInterval` the pulse frequency (`platPPS`/ `vertPPS`) for each motor is incremented and the timer is reset. 

### `runprint()`
The function 'runprint()' controls the platform motor similarly to `runspd()`. The vertical motor is only activated when `dropBool == true`. `dropBool` is emitted from the function `stepPlatform`, when the platform has completed one full rotation `dropBool` will become `true`. The platform will then be pulsed with a constant period until the desired position is reached. When the desired position is reached `dropBool` becomes `false`. 

### `slowAll()`
The function `slowAll()` decrements the pulses period for both motors until they are stopped. The drivers are pulsed with a certain period, because the two motors have different speeds they are pulsed seperately. `slowAll` works similarly to `runspd`, with the exception that it increments the pulse period until the motors are practically stopped after which the `DisableAll` mode is activated.

### `homing()`
When the function `homing` is called, first the top end switch is checked to see if it is not active. When the end switch is not active the vertical direction will be set upward and the vertical motor will be pulsed until the switch is pressed. When the switch is pressed the verttical motor will stop and reverse the direction. The the motor will drop until the switch is released. When the switch is released the `DisableAll` mode is activated.

### `calcRampi()`
The function `calcRampi()` every time a new command is received. The motors ramp up and down by incrementing/decrementing the pulse frequency. This is done in fixed steps, `calcRampi()` calculates these steps. 


