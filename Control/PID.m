function y = PID(u)

persistent errorSum
persistent PrevDerivative
persistent ept
persistent Temp
Tsample = 1/500;
nhat = 16;               % Number of samples taken/kept
P = 4/(Tsample*nhat);                 % Pole
KP = 10;
KI = 5;
KD = 2;

% Initialize Temp
if isempty(Temp)
    Temp = 0;
end

% Initialize ept
if isempty(ept)
    ept = zeros(1, nhat);
    total = 0;
    for i = 1:nhat
        calc = P*exp(-P*i*Tsample);
        ept(i) = calc;
        total = total + calc;
    end
    scale = 1/(Tsample*total);
    ept = ept*scale;
end

% Initialize errorSum
if isempty(errorSum)
    errorSum = 0;
end

% Initialize PrevDerivative
if isempty(PrevDerivative)
    PrevDerivative = zeros(1,nhat);
end

ProportionalOutput = u*KP;

errorSum = errorSum + u*Tsample;
IntegralOutput = errorSum*KI;

PrevDerivative(1) = (u - Temp);
DerivativeOutput = dot(ept, PrevDerivative)*KD;

% Take a step in time
for n = 1:(nhat - 1)
    PrevDerivative(nhat + 1 - n) = PrevDerivative(nhat - n);
end

Temp = u;

y = ProportionalOutput + IntegralOutput + DerivativeOutput;
end