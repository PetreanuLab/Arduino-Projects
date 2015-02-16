

// Constant definitions
const long timeStep = 200000;  // Timestep for calculating instant speed [micros]
const long loopCycle = 2000;
const char turnStep = 32;  // Number of steps in complete turn (should be power of 2 and less than 1024)
const float turnCm = 8.0;  // Displacement of complete turn [cm]

// Pins section
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int speedInPin = 4;  // Digital input pin for receiving the threshold speed
const int stepOutPin = 8;  // Digital output pin for signaling a step (direction determined by pulse duration)
const int speedOutPin = 7; // Digital output pin for signaling speed over threshold
//
// DO NOT CHANGE FROM HERE UNLESS YOU KNOW WHAT YOU ARE DOING
//

// Auxiliary constants
const byte speedMultiplier = 1000000/(timeStep);  // Inverse of timestep
const int turnResolution = 1024/turnStep;  // Resolution of each step [int units]
const float turnResolutionCm = turnCm/float(turnStep); // Resolution of each step [cm]

// Variable instantiations
int turnValue; // Rotary encoder value [int]      
int turnValueOld; // Old value for detecting if a complete turn was achieved [int]
int turnChange; // Value change between consecutive loops [int]

int turnPosition; // Position value in discrete steps [steps]
int turnPositionOld; // Old position value for detecting if a step in either direction occured [steps]
int turnPositionChange; // Position value change between consecutive loops [steps]

int turnPositionLast; // Position value of last time step for determining displacement [steps]

int nTurns = 0; // Number of turns since beginning [turns]
int nTurnsLast = 0; // Number of turns of last time step for determining displacement [turns]

long time; // Current execution time [micros]
long timeOld;  // Time of previous main loop
long dt;  // Elapsed time between consecutive loops
long timeLast; // Time of last time step for determining speed [micros]
long elapsedTime; // Elapsed time between consecutive time steps [micros]

float displacement; // Displacement between time steps [cm]
float instSpeed; // Instant speed between time steps [cm/s]
volatile unsigned long speedRead = 2000;  // Speed read from the bcontrol pulse [micros x 5]
volatile float thresSpeed = 1.0; // Threshold speed for presenting the stimulus [cm/s]

// Setup 
void setup() {
  // Sets output pins
  digitalWrite(stepOutPin, LOW);
  digitalWrite(speedOutPin, LOW);
  pinMode(stepOutPin ,OUTPUT);
  pinMode(speedOutPin ,OUTPUT);
  // Sets interruptions for new trial and new session
  attachInterrupt(0, newTrial, RISING);
  attachInterrupt(1, newSession, RISING);
  // Gets first measurement of rotary encoder
  turnValueOld = analogRead(analogInPin);
  turnPositionOld = turnValue/turnResolution;
  turnPositionLast = turnPositionOld;
  // Gets first time measurements  
  timeOld = micros();
  timeLast = timeOld;
}
// Reset function at address 0
void(* resetFunc) (void) = 0;

// Main loop
void loop() {
  // Gets execution time and compares with the one of the last main loop 
  time = micros();
  dt = time - timeOld;
  // Waits until a loop cycle has passed
  if (dt > loopCycle){
    // Stores time of current loop  
    timeOld = time;
    // Gets value of the rotary encoder and compares with previous value
    turnValue = analogRead(analogInPin); 
    turnChange = turnValue - turnValueOld;
    turnValueOld = turnValue;
    // Detects if a complete turn occurred 
    // Positive direction
    if (turnChange>512) {  
      nTurns++;  // Increments number of turns
    } 
    else if (turnChange <- 512){  // Negative direction
      nTurns--;  // Decrements number of turns
    }
    // Determines the position in steps and compares with previous value
    turnPosition = turnValue/turnResolution;
    turnPositionChange = turnPosition - turnPositionOld;
    turnPositionOld = turnPosition;
    // Detects if step occurred
    // Positive direction
    if (turnPositionChange == -1 || turnPositionChange == (turnStep-1)) {
      // Sends a 400 micros pulse for each step in the positive direction  
      digitalWrite(stepOutPin, HIGH);
      delayMicroseconds(400);
      digitalWrite(stepOutPin, LOW);      
    } 
    // Negative direction
    else if (turnPositionChange == 1 || turnPositionChange == -(turnStep-1)) {  
      // Sends a 800 micros pulse for each step in the negative direction  
      digitalWrite(stepOutPin, HIGH);
      delayMicroseconds(800);
      digitalWrite(stepOutPin, LOW);
    }
    elapsedTime = time - timeLast;
    // Another time step has occurred
    if (elapsedTime > timeStep) {
      // Updates old time
      timeLast = time;
      // Calculates displacement since last time step
      displacement = float(nTurns-nTurnsLast) * 8 + float(turnPositionLast-turnPosition) * turnResolutionCm;
      // Updates old number of turns and position
      nTurnsLast = nTurns;
      turnPositionLast = turnPosition;
      // Determines instant speed
      instSpeed = displacement * speedMultiplier;
      // Compares instant speed with threshold
      if (instSpeed > thresSpeed) {
        // Signals high speed
        digitalWrite(speedOutPin, HIGH);
      }
      else {
        // Signals low speed
        digitalWrite(speedOutPin, LOW);    
      }
      
    }  
  }
}

// Interrupt function
void newTrial() {
  // Resets speed output
  digitalWrite(speedOutPin, LOW);    
  // Reads speed duration from bcontrol and converts to cm/s
  speedRead = pulseIn(speedInPin, HIGH, 100000);
  thresSpeed = float(speedRead)/2000;
}

// Interrupt function
void newSession() {
  // Resets Arduino
  resetFunc();
}

