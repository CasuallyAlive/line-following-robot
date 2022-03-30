classdef MKR_MotorCarrier < handle
    % The MKR_MotorCarrier class is an organization of functions to facilitate
    % communication between MATLAB and the Arduino MKR WiFi 1010 mounted to
    % the
    %
    % This class can send commands to the Arduino MKR to simulate most of
    % the Arduino's features. Digital pins can be written to and read from
    % in the same way as they are on the Arduino, using the pinMode()
    % command to set the mode, and the digitalRead() and digitalWrite() to
    % read and command values. The analog input pins A1, A2, A5 and A6 can
    % be set to stream data at 100Hz usin the startStream('analog')
    % command. The MKR is also set up to stream data from an 3-axis
    % accelerometer, and that data can be set to stream by calling
    % startStream('imu'). The incoming data streams can be read using the
    % getNewData(<stream>) and getAverageData(<stream>) functions.
    %
    % To create a MKR_MotorCarrier object run the command:
    % robot = MKR_MotorCarrier;
    %
    % To see the different functions belonging to the class, use the
    % command:
    % methods(robot);
    %
    % To see details about the functions belonging to the MKR_MotorCarrier
    % class, run:
    % help robot.<functionName>
    %
    %
    % Connor Olsen
    % Dr. Jacob A. George
    % Dr. Daniel Drew
    % University of Utah, Electrical and Computer Engineering, 2021

    properties
        u; robotIP; robotPORT; ready; sizeofBuffer; tempSignal; tempData;...
            analogSensorBuffer; newAnalogIndex; imuSensorBuffer; newImuIndex;...
            count; plotToggle; analogPlotBuffer; imuPlotBuffer; plotDelay;...
            plotDelayMax; available_streams; streamToDraw; digitalPinBuffer; analogPinBuffer;...
            digitalPinsAvailable; analogPinsAvailable; a1; a2; a3; a4; i1; i2; i3;...
            animation_t; graphBuffer; sizeOfRecordedDataBuffer; RecordedDataBuffer;...
            RDB_Toggle; RDB_Index; channelToDraw; red; green; blue; irBuffer; encoderPos; encoderVel;

    end

    methods


        function obj = MKR_MotorCarrier(varargin)
            obj.robotIP = "192.168.1.100";
            obj.robotPORT = 551;
            obj.sizeofBuffer = 500;
            obj.ready = 0;
            obj.analogSensorBuffer = zeros(4, obj.sizeofBuffer);
            obj.imuSensorBuffer = zeros(3, obj.sizeofBuffer);
            obj.newAnalogIndex = obj.sizeofBuffer;
            obj.newImuIndex = obj.sizeofBuffer;
            obj.tempData = zeros(7,1);
            obj.count = 1;
            obj.plotToggle = 0;
            obj.analogPlotBuffer = zeros(4, 100);
            obj.imuPlotBuffer = zeros(3, 100);
            obj.plotDelay = 1;
            obj.plotDelayMax = 15;
            obj.available_streams = ["analog", "imu"];
            obj.streamToDraw = "";
            obj.channelToDraw = 0;
            obj.digitalPinBuffer = [0;NaN;NaN;NaN;NaN;NaN;0;0;0;0;NaN;NaN;0;0];
            obj.analogPinBuffer = [0;0;NaN;NaN;0;0];
            obj.digitalPinsAvailable = [1, 7, 8, 9, 10, 13, 14];
            obj.analogPinsAvailable = [1 2 5 6];
            obj.animation_t = 0;
            obj.graphBuffer = zeros(7,50);
            obj.sizeOfRecordedDataBuffer = 100000;
            obj.RecordedDataBuffer = zeros(7,obj.sizeOfRecordedDataBuffer);
            obj.RDB_Toggle = 0;
            obj.RDB_Index = 1;
            obj.red = 0;
            obj.green = 0;
            obj.blue = 0;
            obj.irBuffer = [NaN;NaN;NaN;NaN];
            obj.encoderPos = [NaN;NaN];
            obj.encoderVel = [NaN;NaN];
            init(obj);
        end


        function pinMode(obj, pin, io)
            %   PINMODE(pin, "INPUT") sets pin as an input pin
            %
            %   PINMODE(pin, "OUTPUT") sets pin as an output pin

            if nargin < 3
                error("Not enough input arguments");
            end

            if ~isnumeric(pin) || ~ismember(pin,obj.digitalPinsAvailable)
                error(strcat("pinMode can only write to the following pins: ", ...
                    num2str(obj.digitalPinsAvailable)));
            end

            io = string(upper(io));

            if ~ismember(io, ["INPUT", "OUTPUT"])
                error("Unable to set pin. Assigned pin mode must either be 'INPUT' or 'OUTPUT'");
            end

            switch io
                case "INPUT"
                    io = 0;
                case "OUTPUT"
                    io = 1;
            end
            sendOverUDP(obj, 0, pin, io);
        end


        function digitalWrite(obj, pin, val)
            %   DIGITALWRITE(pin, val) writes value val (1 or 0) to
            %   designated pin

            if nargin ~= 3
                error("digitalWrite takes two arguments");
            end

            if ~isnumeric(pin) && ~isnumeric(val)
                error("Inputs must be a numer");
            end

            if ~ismember(pin, obj.digitalPinsAvailable)
                error(strcat("digitalWrite can only write to the following pins: ", ...
                    num2str(obj.digitalPinsAvailable)));
            end

            if ~ismember(val, [0 1])
                error("digitalWrite can only write a value of 1 or 0");
            end
            sendOverUDP(obj, 1, pin, val);
        end

        function echoduration = ultrasonicPulse(obj)
            %ULTRASONICPULSE() returns the echo duration from the
            %ultrasonic sensor

            sendOverUDP(obj, 11, 1, 1)
            pause(0.2);
            echoduration = obj.ultrasonicBuffer(1);
        end
        function piezoTone(obj, period, duration)
            %PIEZOTONE(period, duration) sends a signal to MOTOR 1 for an
            %output tone square wave with PERIOD for DURATION
            sendOverUDP(obj, 12, period, duration)
            pause(duration/1000);
        end
        function reflectanceSetup(obj)
            %Digital pins 7,8,9,10 are reserved for IR reflectance sensor
            %(can adjust this once full set of sensors is implemented)
            %Notes:
            %   Vin must be connected to motor carrier board and LEDON
            %   must be directly connected to 5V
            sendOverUDP(obj,13,1,1)
        end
        function vals = readReflectance(obj)
            sendOverUDP(obj,14,1,1)
            pause(0.05)
            v1 = str2num(obj.irBuffer(1));
            v2 = str2num(obj.irBuffer(2));
            v3 = str2num(obj.irBuffer(3));
            v4 = str2num(obj.irBuffer(4));
            vals = [v1,v2,v3,v4];
        end
        function pinvalue = digitalRead(obj, pin)
            % DIGITALREAD(p) returns the value at digital pin p (1 or 0)

            if ~isnumeric(pin)
                error("Input must be a number");
            end
            if ~ismember(pin, obj.digitalPinsAvailable)
                error(strcat("digitalRead can only read from the following pins: ", ...
                    num2str(obj.digitalPinsAvailable)));
            end

            sendOverUDP(obj, 2, pin, 1);
            pause(.05);
            pinvalue = obj.digitalPinBuffer(pin);
        end

        function [r,g,b] = rgbRead(obj)
            % RGBREAD() returns the value of the rgb color sensor (r,g,b)

            sendOverUDP(obj, 18, 0, 0);
            pause(.1);
            r = obj.red;
            g = obj.green;
            b = obj.blue;
        end


        function pinvalue = analogRead(obj, pin)
            % ANALOGREAD(p) returns the value at analog pin p. To read from
            % pin A1, call the function as analogRead(1);

            if ~isnumeric(pin)
                error("Input must be a number. To read pin A1, call analogRead(1)");
            end
            if ~ismember(pin, obj.analogPinsAvailable)
                error(strcat("digitalRead can only read from the following pins: ", ...
                    num2str(obj.analogPinsAvailable)));
            end
            sendOverUDP(obj, 4, pin, 1);
            pause(.05);
            pinvalue = obj.analogPinBuffer(pin);
        end


        function setRGB(obj, r, g, b)
            % SETRGB(r,g,b)  sets the LED to the rgb value given in the
            % arguments. each value must be between 0 and 255
            if nargin ~=4
                error("Invalid number of arguments")
            end

            if ~isnumeric(r) || ~isnumeric(g) || ~isnumeric(b)
                error("Input arguments must be numbers between 0 and 255")
            end

            if ~ismember(r, 0:255) || ~ismember(g, 0:255) || ~ismember(b, 0:255)
                error("RGB values must be integers between 0 and 255");
            end
            c(1) = string(r);
            c(2) = string(g);
            c(3) = string(b);
            for i = 1:3
                while(strlength(c(i)) < 3)
                    c(i) = strcat("0", c(i));
                end
            end
            sendOverUDP(obj, 5, str2double(strcat("9", c(1), c(2), c(3))), 0);
        end


        function motor(obj, motor, dutyCycle)
            % MOTOR(motor, dutyCycle) writes the dutycycle (-100 - 100)
            % to servo motor (1 - 4)

            if nargin < 3
                error("Not enough input arguments");
            end

            if ~isnumeric(motor) || ~ismember(motor,1:4)
                error("motor must be a integer value between 1 and 4")
            end

            if ~isnumeric(dutyCycle) || ~ismember(dutyCycle, -100:100)
                error("dutyCycle must be an integer value between -100 and 100");
            end
            sendOverUDP(obj, 6, motor, dutyCycle);
        end


        function servo(obj, servo, position)
            % SERVO  writes a value to one of the four servo motors
            % available on the arduino.
            %
            %   SERVO(A,B) writes the position cycle B (0-180) to servo A (1, 2,
            %   3 or 4)

            if nargin < 3
                error("Not enough input arguments");
            end

            if ~isnumeric(servo) || ~ismember(servo, 1:4)
                error("servo must be a integer value between 1 and 4")
            end

            if ~isnumeric(position) || ~ismember(position, 0:180)
                error("position must be an integer value between 0 and 180");
            end
            sendOverUDP(obj, 7, servo, position);
        end


        function startStream(obj, streamType)
            % STARTSTREAM  begins a data stream from the MKR. Data is
            % streamed into a buffer that can be read using the
            % getNewData() and getAverageData() methods.
            %
            % STARTSTREAM(<data_stream>) string <data_stream> determines
            % which type of stream will begin streaming (i.e. "analog")
            if ~isstring(streamType) && ~ischar(streamType)
                error("Input must be a string");
            end

            streamTypeCorrected = lower(streamType); % makes input lowercase

            if ~ismember(streamTypeCorrected, obj.available_streams)
                error(obj.invalidStreamInput(streamType));
            end

            obj.stream(streamTypeCorrected, 1);
        end


        function stopStream(obj, streamType)
            % STOPSTREAM  Stops a data from from the MKR.
            %
            % STOPSTREAM(<data_stream>) string <data_stream> determines
            % which type of stream will stop streaming (i.e. "analog")

            if ~isstring(streamType) && ~ischar(streamType)
                error("Input must be a string");
            end

            streamTypeCorrected = lower(streamType); % makes input lowercase

            if ~ismember(streamTypeCorrected, obj.available_streams)
                error(obj.invalidStreamInput(streamType));
            end

            obj.stream(streamTypeCorrected, 0);
        end


        function values = getAverageData(obj, streamType, varargin)
            % GETAVERAGEDATA(<data_stream>) returns an array of the mean
            %   of the data across the entire <data_stream> buffer
            %
            % GETAVERAGEDATA(<data_stream>, A) returns the average
            %   across the buffer of <data_stream> for the previous A
            %   number of samples

            if nargin == 1
                error("Not enough input arguments")
            end

            if ~isstring(streamType) && ~ischar(streamType)
                error("Input must be a string");
            end

            streamTypeCorrected = lower(streamType); % makes input lowercase

            if ~ismember(streamTypeCorrected, obj.available_streams)
                error(obj.invalidStreamInput(streamType));
            end

            if nargin == 3
                if varargin{1} < 1 || varargin{1} > obj.sizeofBuffer || ~isnumeric(varargin{1}) || ~ismember(varargin{1}, 1:500)
                    error(strcat("Input must be a number between 1 and ", string(obj.sizeofBuffer - 1)));
                end
            end

            if nargin > 3
                error("Too many input arguments")
            end

            switch(streamTypeCorrected)
                case "analog"
                    if nargin == 3 && varargin{1} > 0
                        values = mean(obj.analogSensorBuffer(:,end-varargin{1}:end), 2);
                    else
                        values = mean(obj.analogSensorBuffer, 2);
                    end
                case "imu"
                    if nargin == 3 && varargin{1} > 0
                        values = mean(obj.imuSensorBuffer(:,end-varargin{1}:end), 2);
                    else
                        values = mean(obj.imuSensorBuffer, 2);
                    end
                otherwise
            end
        end


        function values = getNewData(obj, streamType)
            % GETNEWDATA(<streamType>)  returns only the data from the
            % designated streamType that hasn't been pulled previously.
            if nargin == 1
                error("Not enough input arguments");
            end

            streamTypeCorrected = lower(streamType); % makes input lowercase

            if ~ismember(streamTypeCorrected, obj.available_streams)
                error(obj.invalidStreamInput(streamType));
            end
            streamTypeCorrected = lower(streamTypeCorrected);
            switch(streamTypeCorrected)
                case "analog"
                    values = obj.analogSensorBuffer(:,obj.newAnalogIndex + 1:end);
                    obj.newAnalogIndex = obj.sizeofBuffer;
                case "imu"
                    values = obj.imuSensorBuffer(:,obj.newImuIndex + 1:end);
                    obj.newImuIndex = obj.sizeofBuffer;
            end
        end


        function getVoltage(obj)
            % GETVOLTAGE  prints the battery level to the command window

            sendOverUDP(obj, 10, 0, 0);
        end

        function resetEncoder(obj,enc_num)
            sendOverUDP(obj,15,1,enc_num);
        end

        function [val1,val2] = readEncoderPose(obj)
            sendOverUDP(obj,16,1,1);
            pause(0.05);
            val1 = str2num(obj.encoderPos(1));
            val2 = str2num(obj.encoderPos(2));
        end

        function [val1,val2] = readEncoderVel(obj)
            sendOverUDP(obj,17,1,1);
            pause(0.05);
            %disp(obj.encoderVal)
            val1 = str2num(obj.encoderVel(1));
            val2 = str2num(obj.encoderVel(2));
        end
        function livePlot(obj, streamType, varargin)
            % LIVEPLOT(streamType)  displays a live plot of all the values
            % read in on the streamType stream.
            %
            % LIVEPLOT(streamType, A) displays a live plot of the values
            % read in on the streamType stream, channel A.

            if nargin == 1
                error("Not enough input arguments");
            end

            if ~isstring(streamType) && ~ischar(streamType)
                error("Input must be a string");
            end

            streamTypeCorrected = lower(streamType); % makes input lowercase

            if ~ismember(streamTypeCorrected, obj.available_streams)
                error(obj.invalidStreamInput(streamType));
            end

            try
                clear obj.a1 obj.a2 obj.a3 obj.a4 obj.i1 obj.i2 obj.i3
                close all;
            catch
            end


            obj.streamToDraw = streamType;
            if nargin > 2
                if obj.streamToDraw == "imu"
                    error("IMU livePlot does not support second arguments");
                end

                if ~isnumeric(varargin{1})
                    error("Second argument must be a number");
                end

                if ~ismember(varargin{1}, obj.analogPinsAvailable)
                    error(strcat("livePlot can only plot following pins: ", ...
                        num2str(obj.analogPinsAvailable)));
                end
                obj.channelToDraw = varargin{1};
            else
                obj.channelToDraw = 0;
            end

            obj.plotToggle = 1;
            obj.plotDelay = 1;
            switch(obj.streamToDraw)
                case 'analog'
                    obj.a1 = animatedline;
                    obj.a1.Color = 'b'; obj.a1.LineWidth = 1;
                    obj.a2 = animatedline;
                    obj.a2.Color = 'r'; obj.a2.LineWidth = 1;
                    obj.a3 = animatedline;
                    obj.a3.Color = 'g'; obj.a3.LineWidth = 1;
                    obj.a4 = animatedline;
                    obj.a4.Color = 'y'; obj.a4.LineWidth = 1;

                    switch(obj.channelToDraw)
                        case 0
                            legend("Analog 1", "Analog 2", "Analog 5", "Analog 6" ,'Location', 'northwest');
                        case 1
                            legend("Analog 1",'Location', 'northwest');
                        case 2
                            legend('',"Analog 2", 'Location', 'northwest');
                        case 5
                            legend('','',"Analog 5", 'Location', 'northwest');
                        case 6
                            legend('','','',"Analog 6" ,'Location', 'northwest');
                    end

                case 'imu'
                    obj.i1 = animatedline;
                    obj.i1.Color = 'b'; obj.i1.LineWidth = 1;
                    obj.i2 = animatedline;
                    obj.i2.Color = 'r'; obj.i2.LineWidth = 1;
                    obj.i3 = animatedline;
                    obj.i3.Color = 'g'; obj.i3.LineWidth = 1;
                    legend("X", "Y", "Z", 'Location', 'northwest');
            end
        end


        function startRecording(obj)
            % STARTRECORDING begins saving the data being streamed.

            if nargin ~=1
                error("Invalid number of input arguments")
            end
            obj.RDB_Toggle = 1;
        end


        function data = stopRecording(obj)
            % STOPRECORDING Stops saving the data being streamed, and
            % returns the saved data

            if nargin ~=1
                error("Invalid number of input arguments")
            end
            obj.RDB_Toggle = 0;
            data = obj.RecordedDataBuffer(:,1:obj.RDB_Index-1);
            obj.RDB_Index = 1;
            obj.RecordedDataBuffer = zeros(7,obj.sizeOfRecordedDataBuffer);
        end


        function close(obj, varargin)
            % CLOSE  clears the udp input buffer, ends the analog streams,
            % turns off the on-board LED back to blue, and clears the
            % udpport object.

            if isobject(obj.u)
                flush(obj.u, "input");
                stream(obj, "ANALOG", 0);
                stream(obj, "IMU", 0);
                setRGB(obj, 0, 0, 25);
                clear obj.u
            end
        end

    end


    methods(Hidden=true)

        function obj = init(obj, varargin)
            % INIT  connects to the UDP Object. This function is generally
            % only called internally when the costructor is ran.

            attemptCount = 0;
            fprintf("Trying to Connect");
            while obj.ready == 0 && attemptCount < 10
                obj.u = udpport("datagram");
                flush(obj.u, "input");
                sendOverUDP(obj, 9, 0, 0);
                pause(.2);
                if obj.u.NumDatagramsAvailable == 0
                    obj.ready = 0;
                    fprintf(".");
                    attemptCount = attemptCount + 1;
                    pause(.25);
                else
                    flush(obj.u, "input");
                    obj.ready = 1;
                    setRGB(obj, 0, 25, 0);
                    fprintf("\nRobot Connected!\n");
                    configureCallback(obj.u, "datagram", 1, @obj.read);
                end
            end
            if obj.ready == 0
                fprintf("\nMATLAB was unable to connect to the MKR 1010\n");
            end
        end


        function read(obj, varargin)
            % READ  is a callback function designed to be called whenever
            % there is a datagram available in the UDP buffer. This
            % function handles the message appropriately, and never needs
            % to be called by the user

            try
                % Read data and update status
                datagramCount = obj.u.NumDatagramsAvailable;
                uDatagram = read(obj.u, datagramCount);
            catch
                disp('UDP Error!');
            end

            for i = 1:datagramCount
                obj.tempSignal = split(convertCharsToStrings(char(uDatagram(i).Data)),":");

                %switch statement goes here.
                switch(obj.tempSignal(1))
                    case "ANA"
                        obj.tempData = str2double(obj.tempSignal(2:end));
                        if sum(obj.tempData(5:7) ~= 0)
                            obj.imuSensorBuffer = circshift(obj.imuSensorBuffer, -1, 2);
                            obj.imuSensorBuffer(:,end) = obj.tempData(5:7);
                            if obj.newImuIndex > 1
                                obj.newImuIndex = obj.newImuIndex - 1;
                            end
                        end
                        if sum(obj.tempData(1:4) ~= 0)
                            obj.analogSensorBuffer = circshift(obj.analogSensorBuffer,-1,2);
                            obj.analogSensorBuffer(:,end) = obj.tempData(1:4);
                            if obj.newAnalogIndex > 1
                                obj.newAnalogIndex = obj.newAnalogIndex - 1;
                            end
                        end
                        if obj.RDB_Toggle
                            obj.RecordedDataBuffer(:,obj.RDB_Index) = obj.tempData;
                            obj.RDB_Index = obj.RDB_Index + 1;
                        end
                        obj.drawPlot();
                    case "RGB"
                        v = str2double(split(obj.tempSignal(2),','));
                        obj.red = v(1);
                        obj.green = v(2);
                        obj.blue = v(3);
                    case "DIG"
                        index = str2double(obj.tempSignal(2));
                        value = str2double(obj.tempSignal(3));
                        obj.digitalPinBuffer(index) = value;
                    case "ANR"
                        index = str2double(obj.tempSignal(2));
                        value = str2double(obj.tempSignal(3));
                        obj.analogPinBuffer(index) = value;
                    case "US"
                        value = str2double(obj.tempSignal(2));
                        obj.ultrasonicBuffer(1) = value;
                    case "MSG"
                        disp(obj.tempSignal(2:end));
                    case "IR"
                        obj.irBuffer = obj.tempSignal(2:end);
                    case "ENC"
                        obj.encoderPos = obj.tempSignal(2:end);
                    case "ENC_VEL"
                        obj.encoderVel = obj.tempSignal(2:end);
                end
            end
        end


        function drawPlot(obj)
            % DRAWPLOT adds the stream (selected from obj.livePlot) to the
            % animated lines to create a real-time plot. This function
            % never needs to be called by the user

            if obj.plotToggle
                if obj.plotDelay >= obj.plotDelayMax
                    obj.plotDelay = 1;
                    obj.animation_t = obj.animation_t + 1;
                    obj.graphBuffer = circshift(obj.graphBuffer,-1,2);
                    obj.graphBuffer(1:4, end) = obj.getAverageData('analog', obj.plotDelayMax);
                    obj.graphBuffer(5:7, end) = obj.getAverageData('imu', obj.plotDelayMax);
                    try
                        switch(obj.streamToDraw)
                            case 'analog'
                                axis([obj.animation_t-40, obj.animation_t, min(min(obj.graphBuffer(1:4,:)))-10, max(max(obj.graphBuffer(1:4,:)))+10]);
                                switch(obj.channelToDraw)
                                    case 0
                                        addpoints(obj.a1,obj.animation_t, obj.graphBuffer(1,end));
                                        addpoints(obj.a2,obj.animation_t, obj.graphBuffer(2,end));
                                        addpoints(obj.a3,obj.animation_t, obj.graphBuffer(3,end));
                                        addpoints(obj.a4,obj.animation_t, obj.graphBuffer(4,end));
                                    case 1
                                        addpoints(obj.a1,obj.animation_t, obj.graphBuffer(1,end));
                                    case 2
                                        addpoints(obj.a2,obj.animation_t, obj.graphBuffer(2,end));
                                    case 5
                                        addpoints(obj.a3,obj.animation_t, obj.graphBuffer(3,end));
                                    case 6
                                        addpoints(obj.a4,obj.animation_t, obj.graphBuffer(4,end));
                                end
                            case 'imu'
                                axis([obj.animation_t-40, obj.animation_t, min(min(obj.graphBuffer(5:7,:)))-10, max(max(obj.graphBuffer(5:7,:)))+10]);
                                addpoints(obj.i1,obj.animation_t, obj.graphBuffer(5,end));
                                addpoints(obj.i2,obj.animation_t, obj.graphBuffer(6,end));
                                addpoints(obj.i3,obj.animation_t, obj.graphBuffer(7,end));
                            otherwise
                        end
                        drawnow update;
                    catch e
                        % disp(e.message);
                        disp("Plot closed");
                        close all;
                        obj.plotToggle = 0;
                        try
                            clear a1 a2 a3 a4 i1 i2 i3
                        catch
                        end
                    end
                end
                obj.plotDelay = obj.plotDelay + 1;
            end
        end


        function sendOverUDP(obj, command, pin, value)
            % SENDOVERUDP  sends a string over UDP to the established UDP
            % host. This isn't designed to used directly by the user, but
            % rather by this class to streamline communication. The command
            % argument is an arbitrarily assigned value that is given
            % meaning on the matlab side to dictate actions to take, while
            % the pin and value arguments generally correspond to pins and
            % values for those pins on the arduino.
            %
            % SENDOVERUDP(A,B,C) sends a string of the form "A:B:C"

            if ischar(command) == 0
                command = int2str(command);
            end
            if ischar(pin) == 0
                pin = int2str(pin);
            end
            if ischar(value) == 0
                value = int2str(value);
            end
            dataString = strcat(command, ':', pin, ':', value);

            write(obj.u, dataString, obj.robotIP, obj.robotPORT); %Sends the data over UDP
        end


        function stream(obj, streamType, on_off)
            % STREAM  streams data from the indicated data stream to the
            % buffer in MATLAB.
            %
            %   STREAM(<data_stream>, 1) begins streaming data from data_stream.
            %   These values can be read directly from the buffer or using
            %   the functions getAverageData() and getNewSensoData()
            %
            %   STREAM(<data_stream>, 0) stops streaming from
            %   <data_stream>

            streamType = lower(streamType); % makes input lowercase

            switch(streamType)
                case "analog"
                    stream = 0;
                case "imu"
                    stream = 1;
                otherwise
                    streams = "";
                    for i=1:length(obj.available_streams)
                        if i == length(obj.available_streams)-1
                            streams = strcat(streams, obj.available_streams(i), " ");
                        elseif i ~= length(obj.available_streams)
                            streams = strcat(streams, obj.available_streams(i),", ");
                        else
                            streams = strcat(streams, "and ", obj.available_streams(i), ".");
                        end
                    end
                    error(strcat("An invalid stream was given. Available streams are ", streams));
            end
            sendOverUDP(obj, 8, stream, on_off);
        end


        function EM = invalidStreamInput(obj, givenInput)
            streams = [];
            for i=1:length(obj.available_streams)
                if i == length(obj.available_streams)-1
                    streams = strcat(streams, "'", obj.available_streams(i), "' ");
                elseif i ~= length(obj.available_streams)
                    streams = strcat(streams, "'", obj.available_streams(i),"', ");
                else
                    streams = strcat(streams, "and '", obj.available_streams(i), "'.");
                end
            end
            EM = strcat("'", givenInput, "' is not a valid stream. Available streams are ", streams);
        end
    end
end

