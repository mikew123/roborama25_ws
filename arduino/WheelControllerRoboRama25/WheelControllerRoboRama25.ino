
#define _PWM_LOGLEVEL_        0
#include "RP2040_PWM.h"
#include "pio_encoder.h"
#include <Servo.h>

// Can be included as many times as necessary, without `Multiple Definitions` Linker Error
#include "RPi_Pico_TimerInterrupt.h"

// Instance RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);

// Instance Claw servos for Left and Right
Servo myservoL;
Servo myservoR;

//#define PI 3.14159265

// Motor PWM frequency
float frequency = 5000;

// WHEEL 
//#define WHEEL_RADUIS_M (0.080/2)
#define WHEEL_RADUIS_M (0.084/2)
#define WHEEL_CIRCUMFERENCE_M (2*PI*WHEEL_RADUIS_M)
#define WHEEL_ENCODER_COUNT_PER_ROTATION (48*20.4086666)
//#define WHEEL_DIST0_RADIUS_M (0.160)
#define WHEEL_DIST0_RADIUS_M (0.275)

// ODOM POD
#define ODOM_RADUIS_M (0.048/2)
#define ODOM_CIRCUMFERENCE_M (2*PI*ODOM_RADUIS_M)
#define ODOM_ENCODER_COUNT_PER_ROTATION (2000.0)


// global variables that can be updated over serial interface
int EncoderSendRateHz = 1; // msg= OR Hz
float wheelFwdOffsetCal = 1.000; // msg= WO fwd rev
float wheelRevOffsetCal = 1.000;
float wheelAccellerationMaxMpsPs = 1.0;


// not yet updatable

const float PidPropCoef[2] = {-2.0, -2.0};
const float PidIntCoef[2] = {-0.1, -0.1};

//#define PIN0 0 /* I2C */
//#define PIN1 1 /* I2C */
//#define PIN8 26
//#define PIN8 27

// Wheel motor odometry pins
#define PIN_MOTOR_ODOM_RA 4
#define PIN_MOTOR_ODOM_RB 5
#define PIN_MOTOR_ODOM_LA 2
#define PIN_MOTOR_ODOM_LB 3

// Wheel motor driver pins
#define PIN_MOTOR_L1      6
#define PIN_MOTOR_L2      7
#define PIN_MOTOR_R1      14
#define PIN_MOTOR_R2      15


// Odometry Pod odometry pins
#define PIN_POD_ODOM_X1 10
#define PIN_POD_ODOM_X2 11
#define PIN_POD_ODOM_Y1 12
#define PIN_POD_ODOM_Y2 13

#define PIN_SERVO_L  8
#define PIN_SERVO_R  9
#define PIN_LIMSW_L 28
#define PIN_LIMSW_R 29

//creates pwm instance for motor drive
RP2040_PWM* PWM_MOTOR_L1;
RP2040_PWM* PWM_MOTOR_L2;
RP2040_PWM* PWM_MOTOR_R1;
RP2040_PWM* PWM_MOTOR_R2;

// Odometry encoder encoders (Input even pin of encoder pin pairs)
PioEncoder encoder[4] = {PIN_MOTOR_ODOM_RA, PIN_MOTOR_ODOM_LA, PIN_POD_ODOM_X1, PIN_POD_ODOM_Y1};
enum {
  encoderR_e = 0, // right wheel
  encoderL_e = 1, // left wheel
  encoderX_e = 2, // X direction Odom Pod
  encoderY_e = 3  // Y direction Odom Pod (angle)// Alternate way servo.writeMicroseconds(us) may aloow > 0-180
};

int encoderPolarity[4] = {-1,1,-1,1};
int32_t currEncoderCount[4] = {0,0,0,0};

int32_t lastEncoderCount[2] = {0,0};
int16_t diffEncoderCount[2] = {0,0};
double currTravelMeters[2] = {0,0};
double diffTravelMeters[2] = {0,0};
double velocityMPS[2] = {0,0};
uint32_t currTimeUs = 0;
uint32_t lastTimeUs = 0;
uint32_t diffTimeUs = 0;

// Min and Max 180 movement for the 2 servos used
#define clawUsecMinL  700 /*600 /* CLOSED */
#define clawUsecMaxL 2550 /*2300 /*2150 /* OPENED */

#define clawUsecMinR  400 /*650 /*750 /* OPENED */
#define clawUsecMaxR 2375 /*2300 /* CLOSED */

void InitServos() {
  myservoL.attach(PIN_SERVO_L, clawUsecMinL, clawUsecMaxL);
  myservoR.attach(PIN_SERVO_R, clawUsecMinR, clawUsecMaxR);
  OperateClaws(0,0);
  // Init limit switches
  pinMode(PIN_LIMSW_L, INPUT_PULLUP);
  pinMode(PIN_LIMSW_R, INPUT_PULLUP);

}

// Repeat calls in loop until true is returned
// Claws can be operated 0 to 100 percent at a set time interval
// Returns currently operating true/false

// persistant variables
uint32_t clawStopTime = 0; // time clow positiong stops
uint32_t clawNextTime = 0; // next time to make a claw movement
float clawCurrentPct = 0; // claws current percent position
float clawChangePct = 0;

bool OperateClaws(int clawDesiredPct, int clawPeriodMs) {
  bool operating = true;
  float dt = 10.0; // 10 ms constant update period msec
  uint32_t now = millis();

  // Read limit switches
  bool limSwL = !digitalRead(PIN_LIMSW_L);
  bool limSwR = !digitalRead(PIN_LIMSW_R);

  // Is the claw opening or closing?
  bool closing = clawCurrentPct < clawDesiredPct;


  // limit operating time 0 to 10 sec
  if (clawPeriodMs<0) clawPeriodMs = 0;
  if (clawPeriodMs>10000) clawPeriodMs = 10000;

  // initialize new claw positioning, exe once for each claw positioning
  if (clawNextTime==0) {
    clawStopTime = now + clawPeriodMs;
    clawNextTime = now;
    clawChangePct = clawDesiredPct - clawCurrentPct;
  } 
  
  // Operate claw until time finished or closing with can secured 
  if (now >= clawNextTime) {

    // Slow down claw closing when it gets close
    if(closing && clawCurrentPct>=80) dt = dt/100;

    // Limit claw range 0 to 100 percent
    if (clawDesiredPct<0) clawDesiredPct=0;
    if (clawDesiredPct>100) clawDesiredPct=100;

    if (clawDesiredPct != int(clawCurrentPct)) {
      // determine claw position for this time interval
      if(clawPeriodMs==0) { 
        // instant claw positioning
        clawCurrentPct = float(clawDesiredPct);
      } else {
        clawCurrentPct += clawChangePct / (float(clawPeriodMs)/dt);
      }
    }

    int clawUsecRangeL = clawUsecMaxL - clawUsecMinL;
    int clawUsecL = clawUsecMaxL - clawUsecRangeL*clawCurrentPct/100;
    int clawUsecRangeR = clawUsecMaxR - clawUsecMinR;
    int clawUsecR = clawUsecMinR + clawUsecRangeR*clawCurrentPct/100;

    // Stop closing Left claw when Right limit switch is triggered
    if(!(closing && limSwR)) {
      myservoL.writeMicroseconds(clawUsecL);
//      Serial.print("Operating L "); Serial.println(closing);
    }

    // Stop closing Right claw when Leftblimit switch is triggered
    if(!(closing && limSwL)) {
      myservoR.writeMicroseconds(clawUsecR);
//      Serial.print("Operating R "); Serial.println(closing);
    }

    if(closing && limSwL && limSwR) {
      // Back off to reduce servo chatter
      clawCurrentPct-=4;
      clawUsecL = clawUsecMaxL - clawUsecRangeL*clawCurrentPct/100;
      clawUsecR = clawUsecMinR + clawUsecRangeR*clawCurrentPct/100;
      Serial.println("Claw backing off 4%");
      myservoL.writeMicroseconds(clawUsecL);
      myservoR.writeMicroseconds(clawUsecR);
      clawStopTime = 0;
      clawNextTime = 0;
      operating = false;
    } 
    else if (clawPeriodMs>0) {
      clawNextTime =  now + dt;
      operating = true;
    } 
    else {
      clawStopTime = 0;
      clawNextTime = 0;
      operating = false;
    }
  } 
  else if (now >= clawStopTime) {
    clawStopTime = 0;
    clawNextTime = 0;
    operating = false;
  }

// DEBUG
// Serial.print("P="); Serial.print(clawCurrentPct);
// Serial.print(" C="); Serial.print(closing);
// Serial.print(" D="); Serial.print(dt);
// Serial.print(" L="); Serial.print(limSwL);
// Serial.print(" R="); Serial.println(limSwR);

  // Serial.print(operating); Serial.print(" ");
  // Serial.print(now);Serial.print(" ");
  // Serial.print(clawNextTime);Serial.print(" ");
  // Serial.print(clawStopTime);Serial.print(" ");
  // Serial.print(clawPeriodMs);Serial.print(" ");
  // Serial.print(clawChangePct);Serial.print(" ");
  // Serial.println(clawCurrentPct);

  return operating;
}

void ResetEncoderValues() {
  currTimeUs = micros();
  lastTimeUs = currTimeUs;
  diffTimeUs = 0;

  for(int enc=0; enc<4; enc++) {
    currEncoderCount[enc] = 0;
  }

  // only R and L wheel encoders
  for(int enc=0; enc<2; enc++) {
    lastEncoderCount[enc] = 0;
    diffEncoderCount[enc] = 0;
    currTravelMeters[enc] = 0;
    diffTravelMeters[enc] = 0;
    velocityMPS[enc] = 0;
  }
}

// wheel velocity in Meters Per Second
// WheelMps{Right, Left} = {0,0}
volatile float WheelMps[2] = {0.0, 0.0};
volatile float WheelMpsAccLimited[2] = {0.0, 0.0};
// Wheel motor drive percents initial
volatile float MotorPct[2] = {0.0,0.0};
const float WheelMpsMax = 1.0; // +- max

// PID parameter and integral
volatile float PidErrorMps[2] = {0.0, 0.0};
volatile float PidPropPct[2] = {0.0, 0.0};
volatile float PidIntPct[2] = {0.0, 0.0};
const float PidIntPctMax = 100.0; // +- max


// Read encoders using an ISR timer of 10us
bool timerInterruptHandler(struct repeating_timer *t) {
//void updateEncoderCountsISR() {
  (void) t;   

  // read wheel and pod encoders
  currTimeUs = micros();
  for(int enc=0; enc<4; enc++) {
    currEncoderCount[enc] = encoder[enc].getCount() * encoderPolarity[enc];
  }

  // Time from last encoder read
  diffTimeUs = currTimeUs - lastTimeUs;
  lastTimeUs = currTimeUs;

  // Process R and L wheel encoder values 
  for(int enc=0; enc<2; enc++) {
    diffEncoderCount[enc] = currEncoderCount[enc] - lastEncoderCount[enc];
    lastEncoderCount[enc] = currEncoderCount[enc];
    if((enc==encoderR_e) || (enc==encoderL_e)) {
      currTravelMeters[enc] = (currEncoderCount[enc]/WHEEL_ENCODER_COUNT_PER_ROTATION)*WHEEL_CIRCUMFERENCE_M;
      diffTravelMeters[enc] = (diffEncoderCount[enc]/WHEEL_ENCODER_COUNT_PER_ROTATION)*WHEEL_CIRCUMFERENCE_M;
      velocityMPS[enc] = diffTravelMeters[enc]/(1e-6*diffTimeUs);
    } else {
      currTravelMeters[enc] = (currEncoderCount[enc]/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
      diffTravelMeters[enc] = (diffEncoderCount[enc]/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
      velocityMPS[enc] = diffTravelMeters[enc]/(1e-6*diffTimeUs);
    }
  }


  // PID loop to regulate R and L wheel speed based on wheel encoder values
  for(int enc=0; enc<2; enc++) {
    // Limit acceleration and decelleration of wheel velocity
    float targetMps = WheelMps[enc];
    float limitedMps = WheelMpsAccLimited[enc];
    float dMpsAcc = wheelAccellerationMaxMpsPs * (diffTimeUs/1e6);

    if(limitedMps != targetMps) {
      if(limitedMps < targetMps) {
        limitedMps = limitedMps + dMpsAcc;
        if(limitedMps > targetMps) limitedMps = targetMps;
      } else if(limitedMps > targetMps) {
        limitedMps = limitedMps - dMpsAcc;
        if(limitedMps < targetMps) limitedMps = targetMps;
      } 
      WheelMpsAccLimited[enc] = limitedMps;
    }

    // error is positive when measured velocity > target velocity
    PidErrorMps[enc] = velocityMPS[enc] - limitedMps;
    PidPropPct[enc] = PidErrorMps[enc] * PidPropCoef[enc];
    PidIntPct[enc] += PidErrorMps[enc] * PidIntCoef[enc];
    if(PidIntPct[enc] > +PidIntPctMax) PidIntPct[enc] = +PidIntPctMax;
    if(PidIntPct[enc] < -PidIntPctMax) PidIntPct[enc] = -PidIntPctMax;

    float pct = 100*(PidPropPct[enc] + PidIntPct[enc]);
    // TODO: limit pct here? or count on limit check in MotorDrivePct()
    MotorPct[enc] = pct;
  }

  // Set wheel motor drive PWM percent
  MotorDrivePct(MotorPct[1], MotorPct[0]);

  return true;
}


void InitEncoders() {
  for(int enc=0; enc<4; enc++) {
    encoder[enc].begin();
    encoder[enc].reset();
  }

  ResetEncoderValues();
}

void InitMotorDrivers(){
  // initialize to stop-braked (100PCT FOR STOP BRAKED)
  PWM_MOTOR_L1 = new RP2040_PWM(PIN_MOTOR_L1, frequency, 100);
  PWM_MOTOR_L2 = new RP2040_PWM(PIN_MOTOR_L2, frequency, 100);
  PWM_MOTOR_R1 = new RP2040_PWM(PIN_MOTOR_R1, frequency, 100);
  PWM_MOTOR_R2 = new RP2040_PWM(PIN_MOTOR_R2, frequency, 100);
}

// +pct = fwd, -pct = rev, 0pct = stop-braked
// pct limited to +-100%
//void MotorDrivePct(int pctL, int pctR) {
void MotorDrivePct(float pctL, float pctR) {
  pctL = -pctL; // opposite rotation as Right Wheel
  // Limit to +-100 pct
  if(pctL>100) pctL=100;
  if(pctL<-100) pctL=-100;
  if(pctL>=0) {
    // Motor Left Forward
    PWM_MOTOR_L1->setPWM(PIN_MOTOR_L1, frequency, 100);
    PWM_MOTOR_L2->setPWM(PIN_MOTOR_L2, frequency, 100-pctL);
  } else {
    // Motor Left Reverse
     PWM_MOTOR_L1->setPWM(PIN_MOTOR_L1, frequency, 100+pctL);
     PWM_MOTOR_L2->setPWM(PIN_MOTOR_L2, frequency, 100);
  }

  pctR = +pctR;
  // Limit to +-100 pct
  if(pctR>100) pctR=100;
  if(pctR<-100) pctR=-100;
  if(pctR>=0) {
    // Motor Right Forward
    PWM_MOTOR_R1->setPWM(PIN_MOTOR_R1, frequency, 100);
    PWM_MOTOR_R2->setPWM(PIN_MOTOR_R2, frequency, 100-pctR);
  } else {
    // Motor Right Reverse
    PWM_MOTOR_R1->setPWM(PIN_MOTOR_R1, frequency, 100+pctR);
    PWM_MOTOR_R2->setPWM(PIN_MOTOR_R2, frequency, 100);
  }

}

uint32_t loopLastMillis;
uint32_t rlLastMillis;

float rlToSec = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(155200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }  

  InitEncoders();

  InitMotorDrivers();
  MotorDrivePct(0, 0);

  InitServos();

  // Init ISR timer 10 msec (100 Hz) for encoder reads
  ITimer1.attachInterruptInterval(10000L, timerInterruptHandler);

  loopLastMillis = millis();
  rlLastMillis = millis();

  Serial.println("Robot 2024 motor/encoder module initialized");
}

int clawPct = 0;
int clawPeriodMs = 0;
bool clawsBusy = false;

void loop() {

 if (Serial.available() > 0) {
    // read the incoming string:
    String incomingString = Serial.readStringUntil('\n');

    // parse string "RL 0.5 0.2" Right wheel = 0.5Mps Left wheel = 0.2 Mps
    float Rv, Lv, Tv;
    int Hz;
    float Fwd, Rev;
    float Acc;
    int n1, n2, n3, n4, n5;

    // Check command for RL wheel velocities in meters/sec
    n1 = sscanf(incomingString.c_str(), "RL %f %f %f", &Rv, &Lv, &Tv);
    // Check command for Odometry Encoder sending rate in Hz
    n2 = sscanf(incomingString.c_str(), "OR %d", &Hz);
    // Check command for wheel offset to correct pull to left or right
    n3 = sscanf(incomingString.c_str(), "WO %f %f", &Fwd, &Rev);
    // Check command for wheel acceleration rate
    n4 = sscanf(incomingString.c_str(), "AR %f", &Acc);
    // Check command for claw positioning
    n5 = sscanf(incomingString.c_str(), "CP %d %d", &clawPct, &clawPeriodMs);

    if(n1==3) {
      // Set timeout 
      if (Tv<0.0) Tv=0.0;
      rlToSec = Tv;
      
      // RL wheel velocities
      // compensate for RL offset drift
      if (Rv >= 0) Rv = Rv*wheelFwdOffsetCal;
      else Rv *= wheelRevOffsetCal;
      if (Lv >= 0) Lv = Lv/wheelFwdOffsetCal;
      else Lv *= wheelRevOffsetCal;

      //Limit +- veleocity
      if(Rv > +WheelMpsMax) Rv = +WheelMpsMax;
      if(Rv < -WheelMpsMax) Rv = -WheelMpsMax;
      if(Lv > +WheelMpsMax) Lv = +WheelMpsMax;
      if(Lv < -WheelMpsMax) Lv = -WheelMpsMax;

      WheelMps[encoderR_e] = Rv;
      WheelMps[encoderL_e] = Lv;

      rlLastMillis = millis();

//      Serial.print(Rv);Serial.print(",");Serial.println(Lv);

    } else {
      // Check command for Odometry Encoder sending rate in Hz
      n2 = sscanf(incomingString.c_str(), "OR %d", &Hz);
      // Check command for wheel offset to correct pull to left or right
      n3 = sscanf(incomingString.c_str(), "WO %f %f", &Fwd, &Rev);
      // Check command for wheel acceleration rate
      n4 = sscanf(incomingString.c_str(), "AR %f", &Acc);
        if(n2 == 1) {
        // Update odometry report rate
        // limit rate
        if(Hz>1000) Hz=1000;
        if(Hz<0) Hz=0;

        EncoderSendRateHz = Hz;
        //Serial.print("OR -> "); Serial.println(Hz);

      } else if(n3 == 2) {
        // Update left-right wheel pull offset
        wheelFwdOffsetCal = Fwd;
        wheelRevOffsetCal = Rev;
        //Serial.print("WO -> "); Serial.print(Fwd); Serial.print(", ");Serial.println(Rev);

      } else if(n4 == 1) {
        // update the max wheel acceleration rate
        wheelAccellerationMaxMpsPs = Acc;
        //Serial.print("AR -> "); Serial.println(Acc);

      } else if(n5 == 2) {
        // "CP" operate claws
        clawsBusy = true;
        //Serial.print("CP -> "); Serial.print(clawPct);Serial.print(" "); Serial.println(clawPeriodMs);

      } else {
        Serial.println("Invalid serial command");
      }
    }
  }

  // If no RL commands in configurable timeout then stop
  if ((rlToSec>0.0) && ((millis() - rlLastMillis) > 1000*rlToSec)) {
    WheelMps[encoderR_e]=0;
    WheelMps[encoderL_e]=0;
  }

  // Open and close claws, does not block
  if (clawsBusy==true) clawsBusy = OperateClaws(clawPct, clawPeriodMs);

  // timer for Odometry send rate, does not block loop execution
  if(EncoderSendRateHz>0) {
    uint32_t loopCurrentMillis = millis();

    // TODO: should be >= 1000.0/rate?
    if( (loopCurrentMillis-loopLastMillis) >= (1000/EncoderSendRateHz) ) {
    
      Serial.print("OD "); 
      Serial.print(currTimeUs); 
      Serial.print(" "); Serial.print(currEncoderCount[encoderR_e]);
      Serial.print(" "); Serial.print(currEncoderCount[encoderL_e]);
      Serial.print(" "); Serial.print(currEncoderCount[encoderX_e]);
      Serial.print(" "); Serial.print(currEncoderCount[encoderY_e]);
      Serial.println("");

      loopLastMillis = loopCurrentMillis;
    }
  } 
}

