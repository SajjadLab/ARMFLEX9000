// ELEC 391 ARMFLEX9000
// 16MHz freq
// Input: List of # marshmallows to remove (ex: [1,3])

// Parameters:
#define KP
#define KD
#define KI
double gainA[3];
double gainB[3];
double gainC[3];
double gainD[3];

const double Tsample = 1/500;
const int nhat = 12;
const int P = 4/(Tsample*nhat);

double errorSumA = 0;
double errorSumB = 0;
double errorSumC = 0;
double errorSumD = 0;

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
int PositionDesired = 90;
int M1[] = {};
int M2[] = {};
int M3[] = {};

// Persistent:
double PrevFilter[nhat] = {0};
double PrevDerivative[nhat] = {0};
double ept[nhat];
double Temp = 0; 

void setup() {
  pinMode(InputA, INPUT);
  pinMode(InputB, INPUT);
  pinMode(InputC, INPUT);
  pinMode(InputD, INPUT);
  
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
  // Call ISR
  ISR(InputA, OutputA, gainA[], errorSumA);
}

bool ISR(in, out, gain, errorSum) {
  // Read position (from encoder) and calculate error
  int PositionActual = analogRead(in);
  int error = PositionDesired - PositonActual;

  // Proportional
  double ProportionalOutput = gain[0]*error;

  // Integral
  errorSum += error;
  double IntegralOutput = gain[1]*errorSum;

  // Filtered derivative
  PrevDerivative[0] = error - Temp;
  
  // Derivative output
  double DerivativeOutput = 0;
  for(n=0; n<nhat; n++) {
    DerivativeOutput += PrevDerivative[n]*ept[n];
  }
  
  // Shift PrevDerivative:
  for(n=0; n<=nhat-1; n++){
    PrevDerivative(nhat + 1 - n) = PrevDerivative(nhat - n);
  }
  
  // Update Temp
  Temp = error;

  // Output
  analogWrite(out, ProportionalOutput + IntegralOutput + DerivativeOutput*gain[2]);
  if(error <= 0.1){
    return true;
  }
  return false;
}
