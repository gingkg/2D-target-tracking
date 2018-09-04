% 基于卡尔曼滤波跟踪二维匀加速直线运动目标，在二维平面内，
%当加速度方向和速度方向相同时，即为匀加速直线运动，当加速度方向和
%速度方向不同时，即为曲线运动，也可视为转弯目标
clear all
close all

T = 1; %sample period
FCV = [1 T T^2/2 0 0 0; 0 1 T 0 0 0;0 0 1 0 0 0;
       0 0 0 1 T T^2/2; 0 0 0 0 1 T;0 0 0 0 0 1];
QCV = 1; %Model noise covariance

%Run Time
RunTime = 50; 
%MontoCarlo Times
MonteCarloTimes = 30;

%True Target Measurement
Z_Init = [100 100]';
V_Init = [50 50]';
A_CV   = [0 -10]';

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
   
   P_Init = [1000 0 0 0 0 0; 0 2000 0 0 0 0; 0 0 500 0 0 0;
             0 0 0 1000 0 0; 0 0 0 0 2000 0; 0 0 0 0 0 500];
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

%Drawing Figure
%time step-Position figure
figure
plot(Z_Real(1,1:RunTime),Z_Real(4,1:RunTime),'K');
hold on
plot(Z_Noise(1,1:RunTime),Z_Noise(2,1:RunTime), 'G');
hold on
plot(XCVE(1,1:RunTime),XCVE(4,1:RunTime),'R');
xlabel('X_Position (m)');
ylabel('Y_Position (m)');
title('CVA Model Kalman Filter');
legend('True', 'Measure', 'Filtered','Location','southeast');

%time step-noise compress figure
figure
plot(sqrt((Z_Noise(1,1:RunTime) - Z_Real(1,1:RunTime)).^2+(Z_Noise(2,1:RunTime) - Z_Real(4,1:RunTime)).^2),'K');
hold on
plot(sqrt((XCVE(1,1:RunTime) - Z_Real(1,1:RunTime)).^2+(XCVE(4,1:RunTime) - Z_Real(4,1:RunTime)).^2),'r');
xlabel('Time Step');
ylabel('Noise ComPress (m)');
title('CVA Model Kalman Filter');
legend('Noise', 'Compressed', 'Location','northeast');

%time step-velocity figure
figure
Velocity(1,:) = Z_Real(2,:);
Velocity(2,:) = Z_Real(5,:);
plot([1:RunTime],sqrt((Velocity(1,:)).^2+(Velocity(2,:)).^2),'K');
hold on
plot([1:RunTime],sqrt((XCVE(2,1:RunTime)).^2+(XCVE(5,1:RunTime)).^2),'r');
xlabel('Time Step');
ylabel('Velocity (m/s)');
title('CVA Model Kalman Filter');
legend('Model Speed', 'Filter Speed','Location','southeast');

%time step-Acceleration figure
figure
Acceleration(1,:) = Z_Real(3,:);
Acceleration(2,:) = Z_Real(6,:);
plot([1:RunTime],sqrt((Acceleration(1,:)).^2+(Acceleration(2,:)).^2),'K');
hold on
plot([1:RunTime],sqrt((XCVE(3,1:RunTime)).^2+(XCVE(6,1:RunTime)).^2),'r');
xlabel('Time Step');
ylabel('Acceleration(m/s/s)');
title('CVA Model Kalman Filter');
legend('Model const Acceleration', 'Filter Acceleration','Location','southeast');


%statistics
for i = 1 : RunTime
   Last = ((MonteCarloTimes-1)* RunTime+i);
   X_Mean_ERROR(i)= mean(sqrt(XCVE(1,i:RunTime:Last).^2+XCVE(4,i:RunTime:Last).^2)) - sqrt(Z_Real(1,i)^2+Z_Real(4,i)^2);
   V_Mean_ERROR(i)= mean(sqrt(XCVE(2,i:RunTime:Last).^2+XCVE(5,i:RunTime:Last).^2)) - sqrt(Z_Real(2,i)^2+Z_Real(5,i)^2);
   A_Mean_ERROR(i)= mean(sqrt(XCVE(3,i:RunTime:Last).^2+XCVE(6,i:RunTime:Last).^2)) - sqrt(Z_Real(3,i)^2+Z_Real(6,i)^2);
   
   Z_X_Real = sqrt(Z_Real(1,i)^2+Z_Real(4,i)^2)*ones(1,MonteCarloTimes);
   Z_V_Real = sqrt(Z_Real(2,i)^2+Z_Real(5,i)^2)*ones(1,MonteCarloTimes);
   Z_A_Real = sqrt(Z_Real(3,i)^2+Z_Real(6,i)^2)*ones(1,MonteCarloTimes);
   X_Cov_ERROR(i) = norm(sqrt(XCVE(1,i:RunTime:Last).^2+XCVE(4,i:RunTime:Last).^2)-Z_X_Real)/sqrt(MonteCarloTimes);
   V_Cov_ERROR(i) = norm(sqrt(XCVE(2,i:RunTime:Last).^2+XCVE(5,i:RunTime:Last).^2)-Z_V_Real)/sqrt(MonteCarloTimes);
   A_Cov_ERROR(i) = norm(sqrt(XCVE(3,i:RunTime:Last).^2+XCVE(6,i:RunTime:Last).^2)-Z_A_Real)/sqrt(MonteCarloTimes);
   
   SNA(i) = std(sqrt(XCVE(1,i:RunTime:Last).^2+XCVE(4,i:RunTime:Last).^2)-Z_X_Real)/r;
   
end


figure
subplot(2,3,1);
plot(X_Cov_ERROR,'r-.');
xlabel('Time Step');
ylabel('RMSE');
title('RMSE -Positon');

subplot(2,3,2);
plot(V_Cov_ERROR,'r-.');
xlabel('Time Step');
ylabel('RMSE');
title('RMSE -Velocity');

subplot(2,3,3);
plot(A_Cov_ERROR,'r-.');
xlabel('Time Step');
ylabel('RMSE');
title('RMSE -Acceleration');


subplot(2,3,4);
plot(X_Mean_ERROR,'r-.');
xlabel('Time Step');
ylabel('Mean ERROR');
title('Mean ERROR -Positon');

subplot(2,3,5);
plot(V_Mean_ERROR,'r-.');
xlabel('Time Step');
ylabel('Mean Error');
title('Mean Error -Velocity');

subplot(2,3,6);
plot(V_Mean_ERROR,'r-.');
xlabel('Time Step');
ylabel('Mean Error');
title('Mean Error -Acceleration');

%SNR
figure
plot(SNA,'r-.');
xlabel('Time Step');
ylabel('Signal Noise Ration');
title('CVA Model Kalman Filter');



