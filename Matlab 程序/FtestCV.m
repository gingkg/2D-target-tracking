function [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCV(T,RT,MT, Zinit, Vinit, A)
% 基于卡尔曼滤波跟踪二维匀加速直线运动目标
%T  sample period
FCV = [1 T T^2/2 0 0 0; 0 1 T 0 0 0;0 0 1 0 0 0;
       0 0 0 1 T T^2/2; 0 0 0 0 1 T;0 0 0 0 0 1];
QCV = 1; %Model noise covariance

%Run Time
RunTime = RT; 
%MontoCarlo Times
MonteCarloTimes = MT;

%True Target Measurement
Z_Init = Zinit;
V_Init = Vinit;
A_CV   = A;

for i = 1 : RunTime
   Z_Real(1, i) = Z_Init(1,1) + V_Init(1,1) * (T * i)+ 0.5*A_CV(1,1)*(T * i)^2;
   Z_Real(2, i) = V_Init(1,1) + A_CV(1,1)*(T * i);
   Z_Real(3, i) = A_CV(1,1);
   Z_Real(4, i) = Z_Init(2,1) + V_Init(2,1) * (T * i)+ 0.5*A_CV(2,1)*(T * i)^2;
   Z_Real(5, i) = V_Init(2,1) + A_CV(2,1)*(T * i);
   Z_Real(6, i) = A_CV(2,1);
end

%Noise Target Measurement
r = 50;
R = (diag([r r]))^2;

Noise_Add(1,:) = r * randn(1,RunTime * MonteCarloTimes);
Noise_Add(2,:) = r * randn(1,RunTime * MonteCarloTimes);

for i = 1 : MonteCarloTimes
   Run_Start = (1 + (i-1)*RunTime);
   Run_End   = (i * RunTime);
   Z_Noise(1,Run_Start: Run_End) = Z_Real(1,:) + Noise_Add(1,Run_Start:Run_End);
   Z_Noise(2,Run_Start: Run_End) = Z_Real(4,:) + Noise_Add(2,Run_Start:Run_End);
end

XCVE = zeros(6,MonteCarloTimes);

%Filter
for i = 1 : MonteCarloTimes
   X_Init(1,1) = Z_Noise(1,1 + (i-1) * RunTime);
   X_Init(2,1) = Z_Noise(1,1 + (i-1) * RunTime) - Z_Noise(1,1 + (i-1) * RunTime);
   X_Init(2,1) = X_Init(2,1)/T;
   X_Init(3,1) = 0;
   X_Init(4,1) = Z_Noise(2,1 + (i-1) * RunTime);
   X_Init(5,1) = Z_Noise(2,1 + (i-1) * RunTime) - Z_Noise(2,1 + (i-1) * RunTime);
   X_Init(5,1) = X_Init(5,1)/T;
   X_Init(6,1) = 0;
   
   P_Init = [1000 0 0 0 0 0; 0 2000 0 0 0 0; 0 0 100 0 0 0;
             0 0 0 1000 0 0; 0 0 0 0 2000 0; 0 0 0 0 0 100];
   XCVE(:, 1 + (i-1) * RunTime) = X_Init(:,1);
   PCVE = P_Init;
   XCVI = X_Init(:,1);
   PCVI = P_Init;
   Z = zeros(2,1);
  
   for j = 2 : RunTime
      Z = Z_Noise(:, j + (i-1) * RunTime);
    
      [X_E, P_E, S_s, V_v] = KalmanCVA(FCV,QCV,XCVI, PCVI, R, Z);
      
      XCVE(:,j + (i - 1) * RunTime) = X_E;
      XCVI = X_E;
      PCVI = P_E;
   end
             
end 
end