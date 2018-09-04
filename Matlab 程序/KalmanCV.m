function [XCVE, PCVE, Ss, Vv] = KalmanCV(F, B, Q, XCVI, PCVI, R, Z)
%KalmanCV  the Standard Kalman Filter Use CV Model
%Input:
%F      one step transfer Matrix
%B      system noise driver Matrix
%Q      system noise covariance Matrix 系统噪声协方差矩阵
%XCVI   the last state estimate Matrix
%PCVI   the last covariance of the state Matrix
%R      the measurement noise covariance Matrix
%Z      the measurement Matrix
%Output:
%XCVE   the updated state estimate
%PCVE   the updated covariance of the state
%Ss     the  measurement prediction coviance
%Vv     the innovation or measurement residual

XI(1:2,1) = XCVI(1:2,1);
XI(3:4,1) = XCVI(4:5,1);

PI(1:2,1:2) = PCVI(1:2,1:2);
PI(3:4,3:4) = PCVI(4:5,4:5);

XP = F * XI;
PP = F * PI * F' + B * Q * B';
H  = [1 0 0 0;0 0 1 0];
Ss = H * PP * H' + R;
K  = (PP * H')/ Ss;
PE=(eye(4)-K*H)*PP;
Vv = Z - H * XP;
XE = XP + K * Vv;

XCVE = [(XE(1:2,1))' 0 (XE(3:4,1))' 0]';
PCVE = [PE(1:2,1:2)   zeros(2,4)
        0 0 0 0 0 0
        zeros(2,3) PE(3:4,3:4)  zeros(2,1)
        0 0 0 0 0 0];

%End


