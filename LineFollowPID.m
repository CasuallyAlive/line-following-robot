clear; clc; close all; instrreset;

%% Connect to Device
r = MKR_MotorCarrier;
%% PID CONTROL FOR RPM
r.resetEncoder(1);
r.resetEncoder(2);

pause(0.5);

