#include "Arduino_BMI270_BMM150.h"
#include <String.h>
#include <ArduinoBLE.h>

#define BUFFER_SIZE 20

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

/***********************************

IO ASSIGNMENTS

************************************/

// debug port
const int DEBUG_GPIO = 4;

// Left motor driver pins
const int L_IN1 = 5;
const int L_IN2 = 6;

// Right motor driver pins
const int R_IN1 = 9;
const int R_IN2 = 10;

// Encoder pins
const int L_ENC_A = 7;
const int L_ENC_B = 8;

const int R_ENC_A = 11;
const int R_ENC_B = 12;

/***************************

ROBOT DYNAMICS 

****************************/

const int PWM_MAX = 255;
const int PWM_MIN = 0;  

/*

NOTES FOR ANGLE K VALUES:

KP:
KP will change our "error response". Simple as large KP means large PWM command for small angle changes. Need to have low enough KP to not cause overshoot while high enough
to respond to changes in the angle. Generally, can use higher KP and damp it out with KD

KD: (TESTED @ KP = 5, KI = 45)
KD will change our "prediction response" and is something of a damper. Too large KD damps the KP gain excessively and means the integral term will dominate. 
Low KD means we do not damp KP enough and angle errors will cause oscillation. Want to balance between damping out KP but not allowing KI to dominate.
KD ~0.5
Signature: excessive shaking at equilibrium

KI: (TESTED @ KP = 5, KD = 0.7)
KI will change how much we drift and react to error. Overly large KI means we oscillate a lot when theres accumulted error (gain at accumulted error, if that makes sense)
Want to maximize K to reduce drift, but also tune it to prevent overcompensation when error accumulates. KI ~45 was very good for compensation but poor drifting.
Drifting may be remedied by the position control though. KI ~ 35 was not sufficient and KI > ~50 was too oscillatory at large error.
Signature: Lots of oscillation to return to equilibrium when a slight push is applied to the bumper 

*/
// Angle Control loop K values
const float KP = 8.25;    // 7.75    
const float KD = 0.5;      // 0.5    
const float KI = 45; // 50



// Position Control loop K Values
float PC_KP = 0.001; // 0.001 ; 0.0015
float PC_KD = 0.00125;  // 0.00125; 0.002 
float PC_KI = 0.0003; //0.0003 ; 

// Angle deadzone to prevent jitter
const float ANGLE_DEADZONE = 0.0; 

//In case of angle/motor mismatch
const int CONTROL_SIGN = -1; 

// Default to experimental values, but dymanic assignment
int L_DEADBAND = 42;                  // automate on startup: when encoder becomes non-zero, choose that PWM as deadband 
int R_DEADBAND = 42;

// Offset and robot target tilt angle 
float angleOffset = 0;
float targetAngle = -0.7; // default target angle where centre of gravity is.
float targetPosition = 0.0; //default to standing still
float targetHeading = 45.0;
int headingCorrection = 0;
float MAX_ANGLE = 3.0;

/*********************************

GLOBAL INTERNAL VARIABLES

**********************************/

// PWM command
int basePWM = 0;

// IMU and derived variables
float x, y, z;
float accel_angle = 0;
float filtered_angle = 0.0;
float currentPosition = 0.0;
int loopsPassed = 0;
float gyroBiasX = 0; // found using function on startup
float gyroBiasZ = 0;
float integral;
float angleError, derivative;
static unsigned long last_time = 0;

float zAngle=0;


// Position Control vars
float positionDerivative, positionIntegral;
float positionError;
float lastPosition;
unsigned long PC_tNow, PC_tLast;
float PC_dt;
const unsigned long PC_LOOP_DELAY = 40;
int currentCommand = 2;
int previousCommand = 2;
// Slew-limiting and feedforward
float targetPositionSmoothed = 0;
float lastTargetSmoothed = 0;
float TARGET_SLEW_RATE = 900.0;  // counts per second 




// Complementary filter weight
float k = 0.98;

//Encoder Variables
volatile long LEncCount = 0, LastLEncCount=0;
volatile long REncCount = 0, LastREncCount=0;
unsigned long now, lastTime;
int dLEncCount, dREncCount;
float Lrps,Lrpm,Rrps,Rrpm, encoder_dt;

int correctionAngle = 0;

float extraAngle = 0;
float integralCLAMPER = 50000;

// FSM and controls
enum turnStates {
  turnIdle, turn45, turnWait, turn90
};

enum rampStates {
  rampIdle, rampClimb, rampClimb10, rampClimb15, rampWait, rampDescend, rampComplete
};

turnStates turnState = turnIdle;  
rampStates rampState = rampIdle;

int turnDirection = 0;
int TURN_WAIT_TIME = 3000;
unsigned long turnStartTime;

int RAMP_WAIT_TIME = 4000;
unsigned long rampStartTime;

      //allow integral clamp change with multiplier
      float integralWeight = 1.0;

/*********************************

SETUP FUNCTIONS

**********************************/

void calibrateGyroX() {
  const int samples = 500;
  float sum = 0.0;
  float sumZ = 0.0;
  float gx, gy, gz;

  Serial.println("Keep robot still. Calibrating gyro...");

  for (int i = 0; i < samples; i++) {
    while (!IMU.gyroscopeAvailable()) {
      // wait for gyro data
    }
    IMU.readGyroscope(gx, gy, gz);
    sum += gx;
    sumZ += gz;
    delay(5);
  }

  gyroBiasX = sum / samples;
  gyroBiasZ = sumZ / samples;
  Serial.print("gyroBiasX = ");
  Serial.println(gyroBiasX, 6);
  Serial.print("gyroBiasZ = ");
  Serial.println(gyroBiasZ, 6);
}

// Calculate deadband on startup by testing for pwm until encoder value is non zero
void autoDeadband() {
  // increment PWM on each wheel
  // once encoder values are nonzero STOP, save those PWM values into the deadbands 


  // ts broken as hell. Come back to fix later
    Serial.println("Obtaining Deadbands...");

    const int HOLD_TIME_MS = 100;
    const int MOTION_THRESHOLD = 15; 

    int pwm = 0;
    while (true) {
      if (pwm > 200) {
        Serial.println("Deadband search failed — no motion detected");
        break;
      }
      
      setLeftMotor(pwm);
      
      noInterrupts();
      computeEncoderValues();
      long startCount = LEncCount;
      interrupts();
      
      delay(HOLD_TIME_MS);
      
      noInterrupts();
      computeEncoderValues();
      long endCount = LEncCount;
      interrupts();
      
      if (abs(endCount - startCount) > MOTION_THRESHOLD) {
          L_DEADBAND = pwm;
          break;
      }
      pwm++;
  }

  pwm = 0;
  while (true) {
    if (pwm > 200) {
      Serial.println("Deadband search failed — no motion detected");
      break;
    }
    setRightMotor(pwm);
    
    noInterrupts();
    computeEncoderValues();
    long startCount = REncCount;
    interrupts();
    
    delay(HOLD_TIME_MS);
    
    noInterrupts();
    computeEncoderValues();
    long endCount = REncCount;
    interrupts();
    
    if (abs(endCount - startCount) > MOTION_THRESHOLD) {
        R_DEADBAND = pwm;
        break;
    }
    pwm++;
  }


  Serial.print("L/R DEADBANDS: ");
  Serial.print(L_DEADBAND);
  Serial.print(", ");
  Serial.println(R_DEADBAND);

}

void initAccel() {

  //Hold still on startup to obtain the angle offset
  Serial.println("Keep robot still. Zeroing Robot...");

  IMU.readAcceleration(x, y, z);

  accel_angle = atan2(y, z) * 180.0 / PI;
  filtered_angle = accel_angle;

  angleOffset = 0; //-0.955

  Serial.print("Angle offset: ");
  Serial.println(accel_angle, 4);
  
}

/*******************************

MOVEMENT FUNCTIONS

*******************************/


// Function to set the left motor speed and direction
void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(L_IN1, pwm);
    analogWrite(L_IN2, 0);
  } 
  else if (pwm < 0) {
    analogWrite(L_IN1, 0);
    analogWrite(L_IN2, -pwm);
  } 
  else if (pwm == 0) {
    analogWrite(L_IN1, 255);
    analogWrite(L_IN2, 255);
  }
}

// Function to set the right motor speed and direction
void setRightMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(R_IN1, pwm);
    analogWrite(R_IN2, 0);
  } 
  else if (pwm < 0) {
    analogWrite(R_IN1, 0);
    analogWrite(R_IN2, -pwm);
  } 
  else if (pwm == 0) {
    analogWrite(R_IN1, 255);
    analogWrite(R_IN2, 255);
  }
}

//might want to rewrite for smoother about deadzone
int applyDeadband(int pwm, int deadband) {
  if (pwm > 0) {
    return constrain(pwm + deadband, 0, PWM_MAX);
  } 
  else if (pwm < 0) {
    return constrain(pwm - deadband, -PWM_MAX, 0);
  } 
  else {
    return 0;
  }
}


/******************************* 

ENCODER FUNCTIONS

*******************************/

void computeEncoderValues() {
  unsigned long now = micros();
  encoder_dt = (now-lastTime)/1000000.0;
  lastTime=now;
  noInterrupts();
  dLEncCount = LEncCount-LastLEncCount;
  dREncCount = REncCount-LastREncCount;
  LastLEncCount=LEncCount;
  LastREncCount=REncCount;

  interrupts();

  Lrps = dLEncCount/(1920.0*encoder_dt);
  Rrps = dREncCount/(1920.0*encoder_dt);
  Lrpm = Lrps * 60;
  Rrpm = Rrps * 60;

}

void interruptLeftEncA() {
    if (digitalRead(L_ENC_A) == digitalRead(L_ENC_B)){
        LEncCount++;
    }
    else{
        LEncCount--;
    }
}
void interruptLeftEncB() {
    if (digitalRead(L_ENC_B) == digitalRead(L_ENC_A)){
        LEncCount--;
    }
    else{
        LEncCount++;
    }
}

void interruptRightEncA() {
    if (digitalRead(R_ENC_A) == digitalRead(R_ENC_B)){
        REncCount--;
    }
    else{
        REncCount++;
    }
}
void interruptRightEncB() {
    if (digitalRead(R_ENC_B) == digitalRead(R_ENC_A)){
        REncCount++;
    }
    else{
        REncCount--;
    }
}

/******************************

LOOP SETUP

*******************************/


void setup() {
    
    Serial.begin(115200);


    // Debug pin
    pinMode(DEBUG_GPIO, OUTPUT);
    digitalWrite(DEBUG_GPIO, LOW);
    // H Bridge Pins
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);
    // Encoder Pins
    pinMode(L_ENC_A, INPUT);
    pinMode(L_ENC_B, INPUT);
    pinMode(R_ENC_A, INPUT);
    pinMode(R_ENC_B, INPUT);

    // Attach interrupts to system 
    attachInterrupt(digitalPinToInterrupt(L_ENC_A), interruptLeftEncA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(L_ENC_B), interruptLeftEncB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_A), interruptRightEncA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC_B), interruptRightEncB, CHANGE);

    // Catch for IMU failure. 
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // Initalize and calibrate sensors, error
    integral = 0.0;
    derivative = 0;
    angleError = 0;
    targetPosition = 0;
    lastPosition = 0;
    positionIntegral = 0;
    positionDerivative = 0;
    targetPositionSmoothed = 0;
    lastTargetSmoothed = 0;
    calibrateGyroX();
    delay(1000); // wait a little
    initAccel();
    delay(1000);
    //autoDeadband();


    //BLE Setup
    if (!BLE.begin()) {
      Serial.println("Starting BLE failed!");
      while (1);
    }

    // Set the device name and local name
    BLE.setLocalName("BLE-DEVICEG1");
    BLE.setDeviceName("BLE-DEVICEG1");

    // Add the characteristic to the service
    customService.addCharacteristic(customCharacteristic);

    // Add the service
    BLE.addService(customService);

    // Set an initial value for the characteristic
    customCharacteristic.writeValue("Waiting for data");

    // Start advertising the service
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");

    PC_tLast = micros();
    last_time = micros();

}

/*******************************************


MAIN LOOP


*******************************************/

void loop() {

    // first, read angles
    // next make PWM correctios based on readings
    // repeat 

    digitalWrite(DEBUG_GPIO, HIGH);

    float gyro_x, gyro_y, gyro_z;
    float dt;

    //BLE Setup
    BLEDevice central = BLE.central();
    if (central && (loopsPassed % 2 == 0)) {
      if (customCharacteristic.written()) {
        // Get the length of the received data
          int length = customCharacteristic.valueLength();

          // Read the received data
          const unsigned char* receivedData = customCharacteristic.value();

          // Create a properly terminated string
          char receivedString[length + 1]; // +1 for null terminator
          memcpy(receivedString, receivedData, length);
          receivedString[length] = '\0'; // Null-terminate the string
          //Serial.print(receivedString);

          if(strcmp(receivedString, "FORWARD")==0){
            currentCommand = 0;
          }
          else if(strcmp(receivedString, "BACK")==0){
            currentCommand = 1;
          }
          else if (receivedString[0] =='B'){ //stop
            currentCommand = 2;
          }
          else if (strcmp(receivedString, "LEFT")==0){
            currentCommand = 3; //turn left 45
          }
          else if (strcmp(receivedString, "RIGHT")==0){
            currentCommand = 4; //turn right 45
          } else if (receivedString[0] == 'A') {
            currentCommand = 5; // Ramp function 
          } else if (receivedString[0] == 'C') {
            currentCommand = 6; // RFU (Reserved for future use)
          }
      }

        // Optionally, respond by updating the characteristic's value
        customCharacteristic.writeValue("Data received");
    }

    if (loopsPassed % 2 == 0) {
      if (!IMU.accelerationAvailable()) return;  // if we cant read the acceleration, exit this loop and enter a new one 
    
      // read accelerometer data anc calculate angle
      IMU.readAcceleration(x, y, z);
      accel_angle = atan2(y, z) * 180.0 / PI + angleOffset;

      if (!IMU.gyroscopeAvailable()) return;  // same as accelerometer
      
      IMU.readGyroscope(gyro_x, gyro_y, gyro_z); 
      
      gyro_x -= gyroBiasX; // Remove bias
      gyro_x = -gyro_x; //fix wrong direction of acceleration

      gyro_z = gyro_z-0.149990639;       // .149990639; //magic calibration number //new matthew code for gyro angle
      
      loopsPassed = 0;
    }

    //finding dt
    unsigned long current_time = micros();
    dt = (current_time - last_time) / 1000000.0; 
    last_time = current_time;

    zAngle += gyro_z*dt; //this one for turning //new matthew code for gyro angle

    // complementary angle filter
    filtered_angle = k*(filtered_angle + gyro_x* dt) + (1-k)*accel_angle;

    // Transfer function calculations 
    angleError = filtered_angle - targetAngle;
    derivative = gyro_x;
    integral += angleError * dt;
    integral = constrain(integral, -(PWM_MAX / KI), PWM_MAX / KI);
 
    // Angle Control Loop
    if (abs(angleError) > ANGLE_DEADZONE) {
      basePWM = CONTROL_SIGN * ((KD * derivative) + (KP * angleError) + (KI * integral));
      basePWM = constrain(basePWM, -PWM_MAX, PWM_MAX);
    } else {
      integral = 0;
      basePWM = 0;
    }

    // Position control loop. Runs slower than angle control loop
    PC_tNow = micros();
    if ((PC_tNow - PC_tLast) > PC_LOOP_DELAY*1000) {
      PC_dt = (PC_tNow - PC_tLast) / 1000000.0;
      PC_tLast = PC_tNow;
      computeEncoderValues();
      currentPosition = ((float)LEncCount + (float)REncCount) / 2.0;

      // Slew control
      float targetDelta = targetPosition - targetPositionSmoothed;
      float maxStep = TARGET_SLEW_RATE * PC_dt;
      if (fabs(targetDelta) < maxStep) {
        targetPositionSmoothed = targetPosition;
      } else {
        targetPositionSmoothed += (targetDelta > 0 ? maxStep : -maxStep);
      }

      // feedforward 
      float targetVelocity = (targetPositionSmoothed - lastTargetSmoothed) / PC_dt;
      //Serial.println(targetVelocity);
      lastTargetSmoothed = targetPositionSmoothed;

      positionError = currentPosition - targetPositionSmoothed;

      // Position derivative minus expected velocity
      positionDerivative = ((currentPosition - lastPosition) / PC_dt) - targetVelocity;

      //positionIntegral = constrain(positionIntegral, -((integralWeight*MAX_ANGLE) / PC_KI), ((integralWeight*MAX_ANGLE) / PC_KI));
      positionIntegral = constrain(positionIntegral, -integralCLAMPER, integralCLAMPER);
  
  
      targetAngle = ((PC_KP * positionError) + (PC_KD * positionDerivative) + (PC_KI * positionIntegral));

      // prevent integral from overcompensating 
      if (fabs(targetAngle) < MAX_ANGLE) {
        positionIntegral += positionError * PC_dt; 
      }
      targetAngle = constrain(targetAngle, -MAX_ANGLE, MAX_ANGLE);
    
      lastPosition = currentPosition;
    }

  bool turning = false;
  if (currentCommand == 3 || currentCommand == 4) {
    turning = true;
  } 

  if(loopsPassed % 2 == 0){ 
    if(zAngle > targetHeading + 3){     //adding margin to prevent oscillation
      if (turning) {
        headingCorrection = 10;
      } else {
        headingCorrection = 3;
      }
    } else if (zAngle < targetHeading - 3){
        if (turning) {
          headingCorrection = -10;
        } else {
          headingCorrection = -3;
        }
    } else {
      headingCorrection = 0;
    }
  }

    bool commandChanged = currentCommand != previousCommand;

      // L/R turns, ramp has FSMs and needs different handling
    if (commandChanged) {
      switch(currentCommand) {
        case 3: //left turn
          MAX_ANGLE = 3.0;
          turnDirection = 1; // set the turn direction to be modified by state machine
          turnState = turn45; // init FSM to idle
          break;
        case 4: // right turn. same implementation as above but with opposite direction
          MAX_ANGLE = 3.0;
          turnDirection = -1;
          turnState = turn45;
          break;
        case 5: //ramp
          rampState = rampClimb;
          rampStartTime = millis();
          break;
      }
    }

    
    /*
  
      Control FSMs
      The following FSMs will control the behaviour of the robot during the turns and ramp climb
    
    */


    // TURN FSM
    switch (turnState) {
      case turnIdle: 
        //do nothing. Essentially doubles as a "turn complete"
        break;
      case turn45:
        targetHeading = 45*turnDirection;
        turnState = turnWait;
        turnStartTime = millis();
        break;
      case turnWait:
        if (millis() - turnStartTime >= TURN_WAIT_TIME) {
          turnState = turn90;
          turnStartTime = millis();
        } else turnState = turnWait;
        break;
      case turn90:
        targetHeading = 90*turnDirection;
        if (millis() - turnStartTime >= 5000) {
          turnState = turnIdle;
          turnStartTime = millis();
        } else turnState = turn90;
        break;
      default: turnState = turnIdle;
        break;
    }

    switch(rampState) {
      case rampIdle:
      MAX_ANGLE = 17.0;
      TARGET_SLEW_RATE = 900.0;
      PC_KP = 0.001; // 0.001 ; 0.0015
      PC_KD = 0.00125;  // 0.00125; 0.002 
      PC_KI = 0.0003; //0.0003 ; 
      integralWeight = 1.0;
      //do nothing and restore stable max angle. Return after complete
        break;
        case rampClimb:
          TARGET_SLEW_RATE = 1800.0;
          targetPosition = 7900;
          integralWeight = 0.5;
          MAX_ANGLE = 17.0;

          PC_KI = 0.0003;
          integralCLAMPER = 20000;
        
          if ((LEncCount+REncCount/2.0) > 3900) {
            rampState = rampClimb10; //REVERT
            rampStartTime = millis();
          }
          break;
          
        case rampClimb10:
        integralCLAMPER = 29700;
        //26300; that goes to lower middle of 10 degrees
        //26700 goes to middle upper of 10 deg ramp

        if((LEncCount+REncCount/2.0) > 7400){
          rampStartTime = millis();
            rampState = rampClimb15;
          }
          
          if((LEncCount+REncCount/2.0) < 4000){
            rampState = rampClimb;
          }

          break;
        
        case rampClimb15:
          integralCLAMPER = 47500; //was 31650
           if((LEncCount+REncCount/2.0) > 8000){
            rampState = rampWait;
          }

          if((LEncCount+REncCount/2.0) < 7300){
            rampState = rampClimb15; //keep it the same it needs that boost
          }

          break;
        case rampWait:
        integralCLAMPER = 35000;
          if ((millis() - rampStartTime) > 360000) {
            rampState = rampDescend;
            rampStartTime = millis();
          }
          break;
        case rampDescend:
          targetPosition = 2000;
          integralCLAMPER = 19000; //to slow the wind downhill

          if (millis() - rampStartTime >= RAMP_WAIT_TIME) {
            rampState = rampComplete;
          }
          break;
        case rampComplete:
        targetPosition = 0;
          integralCLAMPER = 6000; //help stop it
          rampState = rampIdle;
          break;
        

      default: rampState = rampIdle;
        break;
    }


    //int turnPWM = 0; // change with remote control. Positive number for right turn, negative for left turn
    // deprecated turn system
    bool homeSet = true; // double check intended functionality
    // command inits
    switch (currentCommand) {
      case 0:           // forward
        MAX_ANGLE = 3.0;
        homeSet = false;
        targetPosition = 3500;
        targetHeading = 0;
        break;
      case 1:           // backwards
        MAX_ANGLE = 3.0;
        homeSet = false;
        targetPosition = -3500;
        targetHeading = 0;
        break;
      case 2:           // stop
        MAX_ANGLE = 16.0; //REVERT
        targetPosition = 0;
        targetHeading = 0;
        targetPositionSmoothed = 0;
        lastTargetSmoothed = 0;
        extraAngle = 0.0; //REVERT
        if (!homeSet) {
          positionError = 0;
          currentPosition = 0;
          REncCount = 0;
          LEncCount = 0;
          homeSet = true;
        }
        break; 
    }

    // Actuation
    int leftPWM = applyDeadband(basePWM + headingCorrection, L_DEADBAND);
    int rightPWM = applyDeadband(basePWM - headingCorrection, R_DEADBAND);
    //int leftPWM = 0;
    //int rightPWM = 0;
    setLeftMotor(leftPWM);
    setRightMotor(rightPWM);

    previousCommand = currentCommand;
    loopsPassed++;

    // Debug prints

    if (loopsPassed % 2 == 0) {
      //Serial.println(filtered_angle);
      //Serial.print("\t");
      // angle loop numbers
      /*Serial.print(KP*angleError);
      Serial.print("\t");
      Serial.print(KD*derivative);
      Serial.print("\t");
      Serial.print(KI*integral);
      Serial.println("");*/
      // position loop numbers
      /*Serial.print(PC_KP*positionError);
      Serial.print("\t");
      Serial.print(PC_KD*positionDerivative);
      Serial.print("\t");
      Serial.print(PC_KI*positionIntegral);
      Serial.print('\t');
      Serial.print(positionIntegral);
      Serial.print('\t');
      Serial.print(targetAngle);
      Serial.print('\t');*/
      //Serial.print(targetPositionSmoothed);
      //Serial.print('\t');*/
      // control stuff
      /*Serial.print(homeSet);
      Serial.print("\t");
      Serial.print(targetPosition);
      Serial.print("\t");
      Serial.print(positionError);
      Serial.print("\t");*/
      //Serial.print(REncCount);
      //Serial.print("\t");
      //Serial.println(LEncCount);

      //Serial.print(leftPWM);
      //Serial.print('\t');
      //Serial.println(rightPWM);
    }
    
    digitalWrite(DEBUG_GPIO, LOW);



}