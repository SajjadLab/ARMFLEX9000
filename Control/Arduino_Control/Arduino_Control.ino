// ELEC 391 ARMFLEX9000
// 16MHz freq
// Input: List of # marshmallows to remove (ex: [1,3])

// Parameters:
#define KP
#define KD
#define KI
const double Tsample = 1/500;
const int nhat = 12;
const int P = 4/(Tsample*nhat);
double errorSum = 0;

// Pins:
int Input = A1;        // Analog input pin for error
int OutputA = A2;      // Analog output pin for the control signal
int OutputB = A3; 
int OutputC = A4; 
int OutputD = A6; 

// Coordinates:
int PositionDesired = 90;
int M1[] = {};
int M2[] = {};
int M3[] = {};

// Persistent:
double PrevFilter[nhat] = {0};
double PrevDerivative[nhat] = {0};
double ept[nhat] = {0};
double Temp = 0; 

void setup() {
  pinMode(Input, INPUT);
  pinMode(OutputA, OUTPUT);
  pinMode(OutputB, OUTPUT);
  pinMode(OutputC, OUTPUT);
  pinMode(OutputD, OUTPUT);

  if(ept == NULL) {
    int total = 0;
    for(i=0; i<=nhat; i++){
      ept(i) = P*exp(-P*i*Tsample);
      total = total + ept(i);
    }
    ept = ept*(1/(Tsample*total))*Tsample;  // Double check
  }
  
}

void loop() {
  
}

int ISR() {
  // Read position (from encoder) and calculate error
  int PositionActual = analogRead(Input);
  int error = PositionDesired - PositonActual;

  // Proportional
  double ProportionalOutput = KP*error;

  // Integral
  errorSum += error;
  double IntegralOutput = KI*errorSum;

  // Filtered derivative
  PrevDerivative[0] = error - Temp;
  // Derivative output
  // Shift PrevDerivative:
  for(n=0; n<=nhat-1; n++){
    PrevDerivative(nhat + 1 - n) = PrevDerivative(nhat - n);
  }
  
  // Update Temp
  Temp = error;

  // Output
  analogWrite(Output, ProportionalOutput + IntegralOutput + DerivativeOutput);
}
