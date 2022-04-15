clear; clc; close all; instrreset;
%% Connect to Device
r = MKR_MotorCarrier;
%% Define boxes, count, and setup for saving data
blockClass = [0 1 2]; % Zero bad, One good, 2 Excellent
trialCount = 25;
saveDirectory = [pwd, '\data'];

%% Collect Data
% Setup places to store data and start data stream
blockCount = length(blockClass);
data = cell(blockCount, trialCount);
r.startStream('analog');

% temp data holders
red = 0;
green = 0;
blue = 0;
ClawSize = 0;
readings = zeros(5, 1);
a = 1;
b = 1;

% Begin gathering data
countdown("Beginning Recording Bad blocks in", 3);
for x = 1:trialCount
    fprintf("Please Place Bad Block, Trial No. %d\n", x);
    pause(10);
    fprintf("reading data");
    [red, green, blue] = r.rgbRead();
    AnalogData = r.getAverageData('analog', 10);
    ClawSize = CloseClaw(r);
    pause(0.2)

    readings(1, 1) = AnalogData(1);
    readings(2, 1) = red;
    readings(3, 1) = green;
    readings(4, 1) = blue;
    readings(5, 1) = ClawSize;

    data{a, b} = readings;
    b = b + 1;
    pause(0.5);
    clc;
end

a = a + 1;
b = 1;
countdown("Beginning Recording Good Block in", 3);
for x = 1:trialCount
    fprintf("Please Place Good Block, Trial No. %d\n", x);
    pause(10);
    fprintf("reading data");
    [red, green, blue] = r.rgbRead();
    AnalogData = r.getAverageData('analog', 10);
    ClawSize = CloseClaw(r);
    pause(0.2)
    
    readings(1, 1) = AnalogData(1);
    readings(2, 1) = red;
    readings(3, 1) = green;
    readings(4, 1) = blue;
    readings(5, 1) = ClawSize;

    data{a, b} = readings;
    b = b + 1;
    pause(0.5);
    clc;
end

a = a +1;
b = 1;
countdown("Beginning Recording Excellent blocks in", 3);
for x = 1:trialCount
    fprintf("Please Place Excellent Block, Trial No. %d\n", x);
    pause(10);
    fprintf("reading data");
    [red, green, blue] = r.rgbRead();
    AnalogData = r.getAverageData('analog', 10);
    ClawSize = CloseClaw(r);
    pause(0.2)
    
    readings(1, 1) = AnalogData(1);
    readings(2, 1) = red;
    readings(3, 1) = green;
    readings(4, 1) = blue;
    readings(5, 1) = ClawSize;

    data{a, b} = readings;
    b = b + 1;
    pause(0.5);
    clc;
end

r.stopStream('analog');  % stop streaming accelerometer
r.close; % disconnect from the MKR
pause(1); % wait a second
clc; % clear command line

if ~exist(saveDirectory, 'dir') %check if a data folder already exists
    mkdir(saveDirectory) %make folder if not
end
filename = ['data_', getenv('COMPUTERNAME'), '_', datestr(now,'yyyy-mm-dd-HH-MM-ss')];
save(filename,'data') %save data

% For Loading Data
% matObj = matfile('nameoffile.mat');
% then do
% data = matObj.data;
% this will read the cell from the struct into data
%%  Calculate Features
Features = zeros(blockCount, trialCount, 5);
for y = 1:blockCount
    for z = 1:trialCount
        blockType = data{y, z};

        Features(y, z, :) = mean(blockType');
    end
end

%% Plot Features
figure(); hold on; grid on;
for a = 1:blockCount
    scatter3(Features(a,:,1), Features(a, :, 2), Features(a,:,3), Features(a,:,4), Features(a,:,5), 'filled');
end

%% Perform Linear Discriminate analysis
TrainingFeatures = reshape(Features, [trialCount*blockCount, 5]);
TrainingLabels = repmat(blockClass, [2, trialCount]);
LDA = fitcdiscr(TrainingFeatures, TrainingLabels);

%% Plot again
figure(); hold on; grid on;
for a = 1:blockCount
    scatter3(Features(a,:,1), Features(a, :, 2), Features(a,:,3), Features(a,:,4), Features(a,:,5), 'filled');
end
limits = [xlim, ylim, zlim];
K = LDA.Coeffs(1, 2).Const;
L = LDA.coeffs(1, 2).Linear;
f = @(x1, x2, x3) K + L(1)*x1 + L(2)*x2 +  L(3)*x3;
h2 = fimplicit(f, limits);
%% Claw Code Function
function size = CloseClaw(obj)
obj.servo(4,0);

previousAnalogVal = 0;
currentAnalogVal = 0;
for i = 10:10:180
   obj.servo(4, i);
   currentAnalogVal = obj.getAverageData('analog', 5);
   pause(0.1);
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
        if (i <= 80 )
          obj.servo(4, i - 23); %s 47 m 35 , B 23
          disp('Big')
          size = i -23;
          break;
        elseif(i > 80 && i < 115)
          obj.servo(4, i - 30);
          disp('Medium')
          size = i -30;
          break;
        else
          obj.servo(4, i - 25);
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
    obj.servo(4, 0);
end