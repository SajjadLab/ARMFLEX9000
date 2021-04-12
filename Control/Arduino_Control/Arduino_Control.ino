// ELEC 391 ARMFLEX9000
// 16MHz freq
// Input: List of # marshmallows to remove (ex: [1,3])
#define TIMER_INTERRUPT_DEBUG 0

// Parameters:
// KP, KI, KD
double gainA[3] = {0.82, 0.008, 0.2};
double gainB[3] = {0.99, 0.01, 0.26};
double gainC[3] = {1, 0.0075, 0.2};
double gainD[3] = {2, 0.1, 0.08};
double ra = 0.13;
double rb = 0.09;
#define RaSquared 0.0169
#define RbSquared 0.0081
#define RaRb2 0.0234   // 2*ra*rb
#define HalfTurn 1.571 // Pi/2

// Timer
#define USE_TIMER_1 true
#define TIMER1_FREQ_HZ 3000

// Libraries
#include <math.h>
#include "TimerInterrupt.h"

const double Tsample = 1/3000;
const int nhat = 8;
const int P = 4/(Tsample*nhat);

// Pins:
int InputA = A1;        // Analog input pin for error
int InputB = A1;
int InputC = A1;
int InputD = A1;

int OutputA = A2;      // Analog output pin for the control signal
int OutputB = A3; 
int OutputC = A4; 
int OutputD = A6; 

// Coordinates:
double marshmallows[][8][3] = {\
{{0.055, 0, 0}, {0, 0.055, 0},     {0, 0.065, 0},     {0, 0.09, 0},     {0, 0.09, HalfTurn},     {0, 0.06, HalfTurn},     {0.055, 0, HalfTurn}, {0.055, 0, 0}}\
, {{0.055, 0, 0}, {0.055, 0.055, 0}, {0.055, 0.065, 0}, {0.055, 0.09, 0}, {0.055, 0.09, HalfTurn}, {0.055, 0.06, HalfTurn}, {0.055, 0, HalfTurn}, {0.055, 0, 0}}\
, {{0.055, 0, 0}, {0.11, 0.055, 0},  {0.11, 0.065, 0},  {0.11, 0.09, 0},  {0.11, 0.09, HalfTurn},  {0.11, 0.065, HalfTurn}, {0.055, 0, HalfTurn}, {0.055, 0, 0}}};

// Persistent:
double PrevDerivativeA[nhat] = {0};
double PrevDerivativeB[nhat] = {0};
double PrevDerivativeC[nhat] = {0};
double PrevDerivativeD[nhat] = {0};
double ept[nhat];
double Temp = 0; 
int completedMarshmallow = 0;
int completedStage = 0;
double errorSumA = 0;
double errorSumB = 0;
double errorSumC = 0;
double errorSumD = 0;

// Functions
void PID(int out, double error, double errorSum, double gain[], double PrevDerivative[]) {
  // Proportional
  double ProportionalOutput = gain[0]*error;

  // Integral
  errorSum += error*Tsample;
  double IntegralOutput = gain[1]*errorSum;

  // Filtered derivative
  PrevDerivative[0] = error - Temp;
  
  // Derivative output
  double DerivativeOutput = 0;
  // Dot product
  for(int n=0; n<nhat; n++) {
    DerivativeOutput += PrevDerivative[n]*ept[n];
  }
  
  // Shift PrevDerivative:
  for(int n=0; n<=nhat-1; n++){
    PrevDerivative[nhat + 1 - n] = PrevDerivative[nhat - n];
  }
  
  // Update Temp
  Temp = error;

  // Output
  analogWrite(out, ProportionalOutput + IntegralOutput + DerivativeOutput*gain[2]);
}


void Controller() {
  // Which and how many marshmallows are we grabbing
  int marshmallowGrab[] = {1, 2, 3};
  int marshmallowCount = 3;

  // Read values
  double alpha_actual = analogRead(InputA);
  double beta_actual = analogRead(InputB);
  double gamma_actual = analogRead(InputC);
  double delta_actual = analogRead(InputD);

  if(completedMarshmallow < marshmallowCount) {
    // If we havent grabbed all the marshmallows then pick
    // completedMarshmallow
    int currentMarshmallow = marshmallowGrab[completedMarshmallow];

    if(completedStage < 8) {
      // Set the target position
      double xc = marshmallows[currentMarshmallow][completedStage][1];
      double yc = marshmallows[currentMarshmallow][completedStage][2];
      double delta = marshmallows[currentMarshmallow][completedStage][3];
      
      // Calculate required angles
      double beta = acos((pow(xc, 2) + pow(yc, 2) - RaSquared - RbSquared)/(RaRb2));

      double alpha = atan(yc/xc) - atan((rb*sin(beta))/(ra + rb*cos(beta)));

      double gamma = (alpha_actual + beta_actual - HalfTurn);

      // Check if we have reached target location (withing 2%)
      double errorAlpha = (alpha_actual - alpha);
      double errorBeta = (beta_actual - beta);
      double errorGamma = (gamma_actual - gamma);
      double errorDelta = (delta_actual - delta);
      
      if(errorAlpha < 0.05 && errorBeta < 0.05 && errorGamma < 0.002 && errorDelta < 0.005) {
        // We have reached target position
        // Increment completedStage
        completedStage++;
      }
      else {
        PID(OutputA, errorAlpha, errorSumA, gainA, PrevDerivativeA);
        PID(OutputB, errorBeta, errorSumB, gainB, PrevDerivativeB);
        PID(OutputC, errorGamma, errorSumC, gainC, PrevDerivativeC);
        PID(OutputD, errorDelta, errorSumD, gainD, PrevDerivativeD);
      }
    }
    else {
      completedStage = 0;
      completedMarshmallow++;
    } 
  }
  else {
    // We are done grabbing, go home and callobrate
    double alpha = 0;
    double beta = 0;
    double gamma = 0;
    double delta = 0;

    // Check if we have reached target location (withing 2%)
    double errorAlpha = (alpha_actual - alpha);
    double errorBeta = (beta_actual - beta);
    double errorGamma = (gamma_actual - gamma);
    double errorDelta = (delta_actual - delta);

    // Call PID
    PID(OutputA, errorAlpha, errorSumA, gainA, PrevDerivativeA);
    PID(OutputB, errorBeta, errorSumB, gainB, PrevDerivativeB);
    PID(OutputC, errorGamma, errorSumC, gainC, PrevDerivativeC);
    PID(OutputD, errorDelta, errorSumD, gainD, PrevDerivativeD);
  }
}

void setup() {
  pinMode(InputA, INPUT);
  pinMode(InputB, INPUT);
  pinMode(InputC, INPUT);
  pinMode(InputD, INPUT);
  
  pinMode(OutputA, OUTPUT);
  pinMode(OutputB, OUTPUT);
  pinMode(OutputC, OUTPUT);
  pinMode(OutputD, OUTPUT);

  // Initialize ept
  if(ept == NULL) {
    int total = 0;
    for(int i=0; i<=nhat; i++){
      ept[i] = P*exp(-P*i*Tsample);
      total = total + ept[i];
    }
    for(int i=0; i<=nhat; i++){
      ept[i] = ept[i]*(1/(Tsample*total))*Tsample;  // Double check
    }
  }
  
  // Initialize interrupt on timer1
  ITimer1.init();
  if (ITimer1.attachInterrupt(TIMER1_FREQ_HZ, Controller))
    Serial.println("Starting  ITimer0 OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer0. Select another freq. or timer");
}

void loop() {
}
