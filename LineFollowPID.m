clear; clc; close all; instrreset;

%% Connect to Device
r = MKR_MotorCarrier;

% CALCULATE THE SAMPLE RATE
vals = 0;

tic
runtime = 1;
r.motor(3,10);

while toc < runtime
%     val = r.readEncoder(1);
    val1, val2 = r.readEncoderPose();
    vals = vals + 1;
end

r.motor(3,0);
disp(vals) %hint: if this is how many samples you got in runtime, then what is the sampling frequency?

%% PID CONTROL FOR RPM
r.resetEncoder(1);
r.resetEncoder(2);

pause(0.5);

vals = 0;
oldval = 0;
val = 0;
motorval = 0;

control = 0; 

tic

rpm_targ = 50;  %The goal RPM

rpms = 0;
times = 0;

error_sum_M1 = 0;
last_error_M1 = 0;
error_sum_M2 = 0;
last_error_M2 = 0;

kp = .05;         %proportional gain
kd = 0.005;        %derivative gain
ki = 0;         %integral gain

runtime = 2;

while toc < runtime
    %count_delta = r.readEncoder(1); %this gets the encoder reading since the last time it was called
    count_delta_M1, count_delta_M2 = r.readEncoderPose();

    %Calculate RPM from count_delta here:
        %hint 1: use the time between samples estimated from the first
        %section
        %hint 2: there are ~720 counts per rotation
        
    time_elapsed = toc;
    if(length(times) ~= 1)
        time_elapsed = times(end) - times(end - 1);
    end    
    rpm_M1 = (count_delta_M1./(time_elapsed)) * (1 / 720) * 60;  %YOU WILL NEED TO EDIT THIS
    rpm_M2 = (count_delta_M2./(time_elapsed)) * (1 / 720) * 60;

%     rpms(end+1) = rpm;
%     times(end+1) = toc;
    
    % Motor 1 (right motor PID values)
    error_M1 = rpm_targ - rpm_M1;
    error_sum_M1 = error_sum_M1 + error_M1;
    error_delta_M1 = last_error_M1 - error_M1;
    last_error_M1 = error_M1;

    % Motor 2 (left motor PID values)
    error_M2 = rpm_targ - rpm_M2;
    error_sum_M2 = error_sum_M2 + error_M2;
    error_delta_M2 = last_error_M2 - error_M2;
    last_error_M2 = error_M2;

    %Write the code for your controller output here, using the gain
    %variables and the three errors computed above:
    control_M1 = control_M1 + error_M1*kp + kd.*error_delta_M1 + ki.*error_sum_M1; %YOU WILL NEED TO EDIT THIS
    control_M2 = control_M2 + error_M2*kp + kd.*error_delta_M2 + ki.*error_sum_M2;

    %Caps the motor duty cycle at +/- 50
    if control_M1 > 50
        control_M1 = 50;
    end 
    if control_M1 < -50
        control_M1 = -50;
    end
    if control_M2 > 50
        control_M2 = 50;
    end
    if control_M2 < -50
        control_M2 = 
    end
    r.motor(3,round(control));
    r.motor(4, round())
end

pause(0.1)
r.motor(3,0)