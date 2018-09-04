function [Z_Real,Z_Noise,XCVE,RunTime,MonteCarloTimes,r]=FtestCV_Turn2(T,RT,MT, Zinit, Vinit, A)
% 跟踪直角转弯的二维匀速直线运动目标
%带比较
%Test the standard kalman Filter use CV Model
%T sample period
FCV = [1 T 0 0; 0 1 0 0;0 0 1 T;0 0 0 1];
BCV = [T^2/2 0; T 0; 0 T^2/2; 0 T];
QCV = 1; %Model noise covariance

%Run Time
RunTime = RT; 
%MontoCarlo Times
MonteCarloTimes = MT;

%True Target Measurement
Z_Init = Zinit;
V_CV   = Vinit;
V_CV2  = [50 -50]';%直角转弯
A_CV   = A;

for i = 1 : 20
   Z_Real(1, i) = Z_Init(1,1) + V_CV(1,1) * (T * i);
   Z_Real(2, i) = V_CV(1,1);
   Z_Real(3, i) = A_CV(1,1);
   Z_Real(4, i) = Z_Init(2,1) + V_CV(2,1) * (T * i);
   Z_Real(5, i) = V_CV(2,1);
   Z_Real(6, i) = A_CV(2,1);
end

for i = 21 : RunTime
   Z_Real(1, i) = Z_Real(1,20) + V_CV2(1,1) * (T * (i-20));
   Z_Real(2, i) = V_CV2(2,1);
   Z_Real(3, i) = A_CV(1,1);
   Z_Real(4, i) = Z_Real(4,20) + V_CV2(2,1) * (T * (i-20));
   Z_Real(5, i) = V_CV2(2,1);
   Z_Real(6, i) = A_CV(2,1);
end


%Noise Target Measurement
r = 50;
R = (diag([50 50]))^2;

Noise_Add(1,:) = r * randn(1,RunTime * MonteCarloTimes);
Noise_Add(2,:) = r * randn(1,RunTime * MonteCarloTimes);

for i = 1 : MonteCarloTimes
   Run_Start = (1 + (i-1)*RunTime);
   Run_End   = (i * RunTime);
   Z_Noise(1,Run_Start: Run_End) = Z_Real(1,:) + Noise_Add(1,Run_Start:Run_End);
   Z_Noise(2,Run_Start: Run_End) = Z_Real(4,:) + Noise_Add(2,Run_Start:Run_End);
end

XCVE = zeros(6,MonteCarloTimes);
cha=0;%目标前后两次采样的速度角之差绝对值

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
   
   P_Init = [1000 0 0 0 0 0; 0 2000 0 0 0 0; 0 0 0 0 0 0;
             0 0 0 1000 0 0; 0 0 0 0 2000 0; 0 0 0 0 0 0];
   XCVE(:, 1 + (i-1) * RunTime) = X_Init(:,1);
   PCVE = P_Init;
   XCVI = X_Init(:,1);
   PCVI = P_Init;

   for j = 2 : RunTime
      Z = Z_Noise(:, j + (i-1) * RunTime);
          %如果cha大于20度角，判断目标发生了转弯，在之前位置的基础上
          %重置参数
          if cha>20
            X_Init(1,1) = X_E(1,1);
            X_Init(2,1) = 0;
            X_Init(3,1) = 0;
            X_Init(4,1) = X_E(2,1);
            X_Init(5,1) =0 ;
            X_Init(6,1) = 0;
            P_Init = [1000 0 0 0 0 0; 0 2000 0 0 0 0; 0 0 0 0 0 0;
             0 0 0 1000 0 0; 0 0 0 0 2000 0; 0 0 0 0 0 0];
            XCVI = X_Init(:,1);
            PCVI = P_Init;
          end
       
      [X_E, P_E, S_s, V_v] = KalmanCV(FCV, BCV, QCV, XCVI, PCVI, R, Z);
      
      XCVE(:,j + (i - 1) * RunTime) = X_E;
      cha=abs(acotd(X_E(5,1)/X_E(2,1))-acotd(XCVE(5,j-1 + (i - 1) * RunTime)/XCVE(2,j-1 + (i - 1) * RunTime)));
      XCVI = X_E;
      PCVI = P_E;
   end
             
end 
end
