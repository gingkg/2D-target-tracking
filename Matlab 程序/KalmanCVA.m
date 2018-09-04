function [XCVE, PCVE, Ss, Vv] = KalmanCVA(F,Q,XCVI, PCVI, R, Z)
%Input:
%F      one step transfer Matrix
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

XI = XCVI;
PI = PCVI;

XP = F * XI;
Q= diag([Q Q Q Q Q Q]);
PP = F * PI * F'+Q;
H  = [1 0 0 0 0 0;0 0 0 1 0 0];
Ss = H * PP * H' + R;
K  = (PP * H')/ Ss;
PE=(eye(6)-K*H)*PP;
Vv = Z - H * XP;
XE = XP + K * Vv;

XCVE = XE;
PCVE = PE;

%End


