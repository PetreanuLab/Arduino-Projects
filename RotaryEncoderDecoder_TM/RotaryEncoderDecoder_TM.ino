

// Constant definitions
const float thresSpeed = 1.0; // Threshold speed for presenting the stimulus [cm/s]
const long timeStep = 200000;  // Timestep for calculating instant speed [micros]
const char turnStep = 32;  // Number of steps in complete turn (should be power of 2 and less than 1024)
const float turnCm = 8.0;  // Displacement of complete turn [cm]

// Pins section
const int analogInPin = A0;    // Analog input pin that the potentiometer is attached to

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

int turnPositionLast = 0; // Position value of last time step for determining displacement [steps]

int nTurns = 0; // Number of turns since beginning [turns]
int nTurnsLast = 0; // Number of turns of last time step for determining displacement [turns]

long time; // Current execution time [micros]
long timeOld; // Execution time of last time step for determining speed [micros]
long elapsedTime; // Elapsed time between consecutive loops [micros]

float displacement; // Displacement between time steps [cm]
float instSpeed; // Instant speed between time steps [cm/s]


// Setup 
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  // Gets first measurement of rotary encoder
  turnValueOld = analogRead(analogInPin);
  turnPositionOld = turnValue/turnResolution;
  // Waits 20 micros
  delayMicroseconds(50);
}

// Main loop
void loop() {
  
  // Gets value of the rotary encoder and compares with previous value
  turnValue = analogRead(analogInPin); 
  turnChange = turnValue - turnValueOld;
  turnValueOld = turnValue;
  // Detects if a complete turn occurred 
  // Positive direction
  if (turnChange>512) {  
    nTurns++;  // Increments number of turns
    // Outputs
    Serial.print("Turns = "); 
    Serial.println(nTurns);
    Serial.println("Positive turn");    
  } 
  else if (turnChange <- 512){  // Negative direction
    nTurns--;  // Decrements number of turns
    // Outputs
    Serial.print("Turns = "); 
    Serial.println(nTurns);
    Serial.println("Negative turn");    
  }
  // Determines the position in steps and compares with previous value
  turnPosition = turnValue/turnResolution;
  turnPositionChange = turnPosition - turnPositionOld;
  turnPositionOld = turnPosition;
  // Detects if step occurred
  // Positive direction
  if (turnPositionChange == -1 || turnPositionChange == (turnStep-1)) {
    // Outputs
    Serial.print("Turn position = ");
    Serial.println(turnPosition, DEC);  
    Serial.println("Positive step");
  } 
  // Negative direction
  else if (turnPositionChange == 1 || turnPositionChange == -(turnStep-1)) {  
    // Outputs
    Serial.print("Turn position = ");
    Serial.println(turnPosition, DEC);  
    Serial.println("Negative step");
  }
  // Gets execution time and compares with the one of the last time step
  time = micros();
  elapsedTime = time - timeOld;
  // Another time step has occurred
  if (elapsedTime > timeStep) {
    // Updates old time
    timeOld = time;
    // Calculates displacement since last time step
    displacement = float(nTurns-nTurnsLast) * 8 + float(turnPositionLast-turnPosition) * turnResolutionCm;
    // Updates old number of turns and position
    nTurnsLast = nTurns;
    turnPositionLast = turnPosition;
    // Determines speed
    instSpeed = displacement * speedMultiplier;
    // Outputs
    Serial.print("displacement = ");
    Serial.println(displacement); 
    Serial.print("elapsed time = ");
    Serial.println(elapsedTime);
    Serial.print("speed = ");
    Serial.println(instSpeed);  
  }  
}
