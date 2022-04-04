% Code for Claw
clear; clc; close all; instrreset;
%% Connect to Device
r = MKR_MotorCarrier;
%% Claw Code
% This is potential code for the claw. It sets up the input/feedback pins
% for the Hall Effect Sensor and the Servo Claw. Currently here are some
% of the assumptions:
% - The magnetic variation of size does not matter and as such we only
%   need to know when its magnetic field is broken, while we use digital
% - The open position is 180 for the Claw

% Setup input pins for claw and hall effect
r.pinMode(13, "INPUT"); % Hall Effect
r.pinMode(6, "INPUT"); % Claw

% Set Claw to open position
r.servo(4, 180);
%r.servo(4, 0);

% this code will run infinitly and hold an object for 5 seconds
% then let go.

while(true)
    % Check Hall Effect to see if box is in front of claw
    HallEffectCheck = r.digitalRead(13);
    if(HallEffectCheck == 1)
        % Begin Closing Claw until feedback says stop
        for i = 180:-5:0
            r.servo(4, i);
            FeedbackCheck = r.digitalRead(6);
            if(FeedbackCheck == 1)
                tic;
                break;
            end
            pause(0.2);
        end

        % Read in color and display it kinda of
        RGB = [0, 0, 0];
        RGB = r.rgbRead();
        disp(RGB)

        %Opens the arm after 5 sec
        if(toc > 5)
            r.servo(4, 0);
            %r.servo(4, 180);
        end
    end
    pause(0.1);
end
