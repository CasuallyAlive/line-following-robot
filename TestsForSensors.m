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