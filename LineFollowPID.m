clear; clc; close all; instrreset;

%% Connect to Device
r = MKR_MotorCarrier;
%% PID CONTROL FOR RPM
r.resetEncoder(1);
r.resetEncoder(2);

pause(0.5);

Vel_M1 = 0;
Vel_M2 = 0;

control_M1 = 0; 
control_M2 = 0;

tic

rpm_targ = 10;  %The goal RPM


error_sum_M1 = 0;
last_error_M1 = 0;
error_sum_M2 = 0;
last_error_M2 = 0;

kp = .05;         %proportional gain
kd = 0.005;        %derivative gain
ki = 0;         %integral gain

runtime = 10;

while toc < runtime
    Vel_M1, Vel_M2 = r.readEncoderVel();

    %Calculate RPM from count_delta here:
        %hint 1: use the time between samples estimated from the first
        %section
        %hint 2: there are ~720 counts per rotation
        
    rpm_M1 = Vel_M1 * (1 / 720) * 60;  
    rpm_M2 = Vel_M2 * (1 / 720) * 60;
    
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
        control_M2 = -50;
    end

    ScaleF = FindSF(control_M1);
    r.motor(3,round(control_M1 * ScaleF));
    r.motor(4, round(control_M2));
end

pause(0.1)
r.motor(3,0)
r.motor(4, 0)


%% Scale Factor function for motor 3 / right motor
function ScaleFactor = FindSF(inputVal)
    if inputVal >= 0 && inputVal < 10
        ScaleFactor = 1.8108;
    elseif inputVal >= 10 && inputVal < 15
        ScaleFactor = 1.7083;
    elseif inputVal >= 15 && inputVal < 20
        ScaleFactor = 1.4024;
    elseif inputVal >= 20 && inputVal < 25
        ScaleFactor = 1.2583;
    elseif inputVal >= 25 && inputVal < 30
        ScaleFactor = 1.13;
    elseif inputVal >= 30 && inputVal < 35
        ScaleFactor = 1.3374;
    else
        ScaleFactor = 1.1556;
    end
end