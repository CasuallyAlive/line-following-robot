% Code for Claw
clear; clc; close all; instrreset;
%% Connect to Device
r = MKR_MotorCarrier;

%% Claw Code
% This is code for the claw. The claw uses analog feedback to determine
% if the servo is trying to close to a position in which it cannot close
% to. It checks the previous analog input, if the current one analog
% feedback within range of it then we are trying to close to position it
% cannot reach and then goes back

r.startStream('analog');
r.servo(4,0);

previousAnalogVal = 0;
currentAnalogVal = 0;
r.livePlot('analog', 2)
for i = 10:10:180
   r.servo(4, i);
   currentAnalogVal = r.getAverageData('analog', 5);
   pause(0.1);
   currentAnalogVal(2)
   previousAnalogVal
    highTresh = 0;
    lowTresh = 0;
   if( i < 50)
        highTresh =  (previousAnalogVal + 0.02);
        lowTresh =  (previousAnalogVal - 0.02);
   else
        highTresh =  (previousAnalogVal + 10);
        lowTresh =  (previousAnalogVal - 10);
   end
   if( (currentAnalogVal(2) <= highTresh ) && (currentAnalogVal(2)>= lowTresh ) )
       try
        %s 47 i 140, m 35 i 110 , B 23 i 80
        if (i <= 70 )
          r.servo(4, i - 23); %s 47 m 35 , B 23
          disp('Big')
          break;
        elseif(i > 70 && i < 115)
          r.servo(4, i - 30);
          disp('Medium')
          break;
        else
          r.servo(4, i - 25);
          disp('Small')
          break;
        end
       catch
           disp('not the end yet')
       end
   end
   
   previousAnalogVal = currentAnalogVal(2);
   pause(0.5);
end
r.stopStream('analog');

%% Print analog values with claw feedback
% This code's purpose is to map a servo position(0-180)
% to an analog input value. The value is an average of ten values
% that is read from the analog stream. This code closes the claw
% servo in steps of five and stores what servo position values
% was used and it analog feedback to the matrix dataMapping.

r.startStream('analog');
r.servo(4, 0);
dataMapping = zeros(37, 2);

row =1;
for i = 0:5:180
    r.servo(4, i);
    feedBack = r.getAverageData('analog', 10);
    dataMapping(row, 1) = i;
    dataMapping(row, 2) = feedBack(2);
    row = row + 1;
    pause(1);
end
%% Claw Code Function
function size = CloseClaw()
r.servo(4,0);

previousAnalogVal = 0;
currentAnalogVal = 0;
r.livePlot('analog', 2)
for i = 10:10:180
   r.servo(4, i);
   currentAnalogVal = r.getAverageData('analog', 5);
   pause(0.1);
   currentAnalogVal(2)
   previousAnalogVal
    highTresh = 0;
    lowTresh = 0;
   if( i < 50)
        highTresh =  (previousAnalogVal + 0.02);
        lowTresh =  (previousAnalogVal - 0.02);
   else
        highTresh =  (previousAnalogVal + 10);
        lowTresh =  (previousAnalogVal - 10);
   end
   if( (currentAnalogVal(2) <= highTresh ) && (currentAnalogVal(2)>= lowTresh ) )
       try
        %s 47 i 140, m 35 i 110 , B 23 i 80
        if (i <= 70 )
          r.servo(4, i - 23); %s 47 m 35 , B 23
          disp('Big')
          size = i -23;
          break;
        elseif(i > 70 && i < 115)
          r.servo(4, i - 30);
          disp('Medium')
          size = i -30;
          break;
        else
          r.servo(4, i - 25);
          disp('Small')
          size = i - 25;
          break;
        end
       catch
           disp('not the end yet')
       end
   end
   
   previousAnalogVal = currentAnalogVal(2);
   pause(0.5);
   end
end