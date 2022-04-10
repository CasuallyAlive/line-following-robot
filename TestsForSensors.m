%Sensor Testing
clear; clc; close all; instrreset;
%% Connect to Device
r = MKR_MotorCarrier;
%% Motors Working for reading pose/sample rate
%Sample Rates for Motors
%Motor 3
Vals_M1 = 0;
Vals_M2 = 0;
VM1 = 0;
VM2 = 0;
counter = 0;
tic
runtime = 10;
r.motor(3,10);
r.motor(4, 10);
while toc < runtime
    Vals_M1, Vals_M2 = r.readEncoderPose();
    [VM1, VM2] = r.readEncoderVel()
    counter = counter + 1;
    pause(0.1);
end
%% Getting value speeds for each motor
RightMotor = zeros(7, 10)
LeftMotor = zeros(7, 10)
VM1 = 0;
VM2 = 0;
x = 1;
pause(0.1);
for i = 10:5:40
    r.motor(3, i);
    r.motor(4, i);
    tic
    y = 1;
    while(toc < 11.2)
        [VM1, VM2] = r.readEncoderVel();
        RightMotor(x, y) = VM1;
        LeftMotor(x, y) = VM2;
        pause(1);
        y = y+1;
    end
    x = x+1;
end
r.motor(3, 0);
r.motor(4, 0);
%% Scale Factor
% Calculate Scale Factor
VRM = abs(RightMotor);
VLM = LeftMotor;

% Mean of every row
VRM_avg = mean(VRM');
VLM_avg = mean(VLM');

% Scale Factor
SF = VLM_avg ./ VRM_avg
%%
SF = 1.13;
for x = 20:10:30
    r.motor(3, round(x * SF))
    r.motor(4, x)
    pause(2);
end
r.motor(3, 0)
r.motor(4, 0)
%% IR sensor Working
r.reflectanceSetup();
%% Separate Setup
pause(0.5);
IR_Readings = [0,0,0,0];
IR_Readings = r.readReflectance();

%% UltraSonic Double check this one
%Look at lab 7 code, 0.0172
%% RGB Sensor
% Test the ensure RGB sensor is reading properly
red = 0;
green = 0;
blue = 0;
red, green, blue = r.rgbRead();
pause(0.5);
RGB = [red, ', ', green, ', ', blue, ', '];
disp(RGB);
%% Hall Effect
% Signals from hall effect sensor to determine if magnetic field is broken
% May or may not use analog for this depending on magnetics used in
% final test, going with digital for now, simpiler. 
r.pinMode(13, "INPUT");

% continously check to see if field is broken
while(true)
    HallEffectSensor = r.digitalRead(13);
    if(HallEffectSensor == 1)
        disp("Magnetic Detected")
    end
    pause(0.5);
end

%% Servo Claw
%Testing to see how to open and close claw using feedback input

% Feedback is digital
r.pinMode(14, "INPUT");

% Set motor to open postion
%r.servo(4, 180);
r.servo(4, 0)
pause(1);
pause on

% Close the servo slowly, if feeback is detected then stop closing servo
for i = 0:1:180
    r.servo(4, i);
    pause(0.1);
    FeedbackCheck = r.digitalRead(14);
    if(FeedbackCheck == 1)
        disp("stop closing")
        break
    end
    disp(i)
    pause(0.1)
end

%% PD controller for the IR sensor


clear; clc; close all; instrreset;


%% PID CONTROL FOR RPM
r.resetEncoder(1);
r.resetEncoder(2);

pause(0.5);

vals = 0;
oldval = 0;
val = 0;
motorval = 0;
Vel_M1 = 0;
Vel_M2 = 0;

control_M1 = 0; 
control_M2 = 0;

tic

error_ir = 10;  % The goal IR val

rpms = 0;

error_sum_M1 = 0;
last_error_M1 = 0;
error_sum_M2 = 0;
last_error_M2 = 0;

kp = .05;         %proportional gain
kd = 0.005;        %derivative gain
ki = 0;         %integral gain
SF = 1.178;

while (1)

    Vel_M1, Vel_M2 = r.readEncoderVel();

    rpm_M1 = Vel_M1 * (1 / 720) * 60;  
    rpm_M2 = Vel_M2 * (1 / 720) * 60;

    
    % Motor 1 (right motor PID values)
    error_M1 = error_ir - rpm_M1;
    error_sum_M1 = error_sum_M1 + error_M1;
    error_delta_M1 = last_error_M1 - error_M1;
    last_error_M1 = error_M1;

    % Motor 2 (left motor PID values)
    error_M2 = error_ir - rpm_M2;
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
    r.motor(3, round(control_M1) *SF);
    r.motor(4, round(control_M2));
end

pause(0.1)
r.motor(3,0)
r.motor(4, 0)
