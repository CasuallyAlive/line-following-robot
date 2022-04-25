classdef SLAM_Controller < handle
    properties
        state; % the state of the robot.
        previousState;
        classifier; % the neural net classifier.
        body; % the motor_carrier object for this SLAM instance.
        %destinations_visited; % nodes in the track graph.
        %forks_visited; % the locations where forks are encountered.
        %forks_completely_traversed; % forks marked as being completely visited.
        track_graph;
        calibration_time;
        max_ir_reading;
        min_ir_reading;
        blinking_rate;
        is_calibrated;
        currentBranch;
        previousBranch;
        branchCheck;
        branchChoice;
        hasBox;
        findingBox;
        boxType;
        %Facing enum?
        MAX_SIZE
        MIN_SIZE
        MAX_HALL
        MIN_HALL
        
    end
    properties (Constant = true)
        KP_RPM = 0.05;
        KD_RPM = 0.005;
        KI_RPM = 0.0;
        
        KP_IR = 1.2;
        KD_IR = 0.005;
        KI_IR = 0.0;
        
        TARGET_RPM = 10;
        
        MAX_RPM = 30;
        TARGET_IR_READING = 0;
        
        R_MOTOR_SF = 0.9;
        
        MOTOR_L = 4;
        MOTOR_R = 3;

        SERVO = 4;

        BAD_BLOCK = 0;
        GOOD_BLOCK = 1;
        EXCELLENT_BLOCK = 2;

        HALL_EFFECT = 1;
        SERVO_ANALOG = 2;

        SF_ULTRASON
    end
    methods
        function obj = SLAM_Controller(body)
            obj.state = States.StandBy;
            obj.previousState = States.StandBy;
            %obj.classifier = classifier;
            obj.body = body;
            obj.max_ir_reading = [nan, nan, nan, nan];
            obj.min_ir_reading = [nan, nan, nan, nan];
            obj.calibration_time = 5;
            obj.blinking_rate = 0.125;
            obj.is_calibrated = false;
            obj.hasBox = false;
            obj.findingBox = true;
            obj.currentBranch = 0;
            obj.previousBranch = 0;
            obj.branchCheck = ones(1,4);
            obj.branchChoice = 1;
            obj.boxType = nan;
        end
        function [posX, posY] = do_task(obj)
            switch(obj.state)
                case States.StandBy
                    obj.previousState = States.StandBy;
                    obj.body.motor(obj.MOTOR_R, 0);
                    obj.body.motor(obj.MOTOR_L, 0);
                    
                    obj.body.resetEncoder(1);
                    obj.body.resetEncoder(2);
                    
                    pause(0.1);
                    
                    if(obj.is_calibrated)
                        obj.state = States.FollowLineForward; %CHANGE
                    else
                        obj.state = States.Calibration;
                        obj.body.setRGB(255,0,0);
                    end
                    posX = 0;
                    posY = 0;
                    return;
                case States.Calibration
                    success = false;
                    while (not(success))
                        success = obj.calibrate_IR_Sensor();
                    end
                    fprintf("Success! The IR sensor has been calibrated.");
                    obj.body.setRGB(0,255,0)
                    obj.state = States.StandBy;
                    obj.is_calibrated = true;
                    
                    posX = 0;
                    posY = 0;
                    return;
                case States.FollowLineForward % PID control for IR sensor and speed
                    disp("followingLine\n");
                    ir_reading = obj.body.readReflectance();
                    pause(0.1)
                    
                    control_M1 = 0; control_M2 = 0;
                    error_IR = 0; prev_error_IR = 0;
                    Threshold = 100;
                    tic;
                    while not(obj.isFork(ir_reading))
                        [error_IR, control_M1, control_M2] = obj.ir_PD_Controller(obj.normalize_IR_reading(ir_reading),prev_error_IR);
                        
                        [control_M1, control_M2] = obj.setMotorSpeeds(control_M1, control_M2);
                        
                        obj.body.motor(obj.MOTOR_R, round(control_M1*obj.R_MOTOR_SF));
                        obj.body.motor(obj.MOTOR_L, round(control_M2));
                        prev_error_IR = error_IR;
                        ir_reading = obj.body.readReflectance();

                        if(not(obj.line_within_proximity(ir_reading,[1,2,3,4])))
                            Threshold = Threshold - 1;
                            if(Threshold <= 0)
                                obj.stopMotors();
                                while(not(obj.line_in_sight(ir_reading,[2,3])))
                                    ir_reading = obj.body.readReflectance();
                                    pause(0.0001);
                                end
                                pause(0.1);
                                Threshold = 100;
                            end
                        else
                            Threshold = 100;
                        end
                    end
                    
                    % Hit fork
                    obj.body.motor(obj.MOTOR_R, 0);
                    obj.body.motor(obj.MOTOR_L, 0);
                    
                    pause(0.01);
                    
                    elapsed_time = toc;
                    
                    obj.state = States.Branches;
                    
                    
                    ;
                case States.Branches                 
                    % check which branch we are in
                    % If we are in zero
                    if(obj.currentBranch == 0)
                        % Do we have a box?
                        if(obj.hasBox)
                            % choose path based on type of box
%                             obj.MoveForward(2);
%                             obj.Turn90DegreeLeft();
%                             pause(2);
%                             obj.TurnToBranchX(1);
%                             obj.previousBranch = obj.currentBranch;
%                             obj.currentBranch = 5;
%                             obj.state = States.FollowLineForward;
                            if(obj.boxType == 0)
                                obj.MoveForward(2);
                                obj.Turn90DegreeLeft();
                                pause(2);
                                obj.TurnToBranchX(2);
                                obj.previousBranch = obj.currentBranch;
                                obj.currentBranch = 5;
                                obj.state = States.FollowLineForward;
                            else
                                obj.MoveForward(2);
                                obj.Turn90DegreeLeft();
                                pause(2);
                                obj.TurnToBranchX(1);
                                obj.previousBranch = obj.currentBranch;
                                obj.currentBranch = 6;
                                obj.state = States.FollowLineForward;
                            end
                        % Else
                        else
                            % choose one of the branches from 1 to 4
                            % Move and turn to that branch
                            obj.MoveForward(2);
                            if(obj.branchCheck(obj.branchChoice) ~= 1)
                                obj.branchChoice = obj.branchChoice + 1;
                            end
                            obj.Turn90DegreeLeft();
                            pause(2);
                            obj.TurnToBranchX(obj.branchChoice);
                            obj.previousBranch = obj.currentBranch;
                            obj.currentBranch = obj.branchChoice;
                            obj.branchChoice = obj.branchChoice + 1;
                            if(obj.branchChoice == 4)
                                obj.branchChoice = 1;
                            end
                            
                            % go to follow line state
                            obj.state = States.FollowLineForward;
                        end
                    % Else if we are in branches 1 to 4
                    elseif(obj.currentBranch >= 1 && obj.currentBranch <= 4)
                        % If we are trying to find box
                        if(obj.findingBox)
                            % go to find box state
                            obj.state = States.GraspItem; % graspItem incomplete needs to move forward
                            % in grasp item we need to set findingBox to
                            % false
                        % Else
                        else
                            obj.ReturnToZero();
                        end
                    % Else we are in branches 5 and 6
                    else
                        % If we have a box
                        if(obj.hasBox)
                            % drop it off
                            obj.state = States.UnGraspItem;
                        % else
                        else
                            % move back to branch zero
                            obj.ReturnToZero();
                        end
                    end
                  ;
                case States.MoveToCenter
                    disp("moving to center \n");
                    obj.MoveForward(2.3);
                    disp(obj.previousState);
                    obj.Turn90DegreeLeft();
                    if(obj.previousState == States.Branches) %% we can add more logic here aka if branch one doesnt have object then go to state branch 2 directly
                        obj.state = States.BranchOne;
                        
                    else
                        
                        obj.state = States.TurnAround;
                    end
                    ;
                case States.GraspItem % Pd? control for grasping the object
                    
%                     disp("I picked a Box");
%                     obj.hasBox = true;
%                     obj.findingBox = false;
%                     obj.MoveForward(2);
%                     obj.state = States.GoBack;
%                     obj.previousState = States.GraspItem;
                    obj.body.startStream('analog');
                    success = false;
                    size = 0;
                    while(not(success))
                        obj.body.servo(obj.SERVO,0);
                        pause(2);
                        [success, block_features] = obj.pickUpAndAnalyzeBlock();
                        if(success == false)
                            distanceDetected = obj.body.ultrasonicPulse() * 0.0172;
                            if(distanceDetected > (obj.SF_ULTRASON * 5 * 2.54))
                                obj.branchCheck(obj.currentBranch) = 0;
                            end
                        end
                    end
                    predicted_block_type = obj.classifier.predict(block_features);
                    obj.boxType = predicted_block_type;
%                     obj.changeState(predicted_block_type);
                    pause(0.1);
                    
                    ;
                case States.TurnAround % Pd? control for rotating the robot a complete 180
                    disp("Turning around \n");
                    tic;
                    obj.body.motor(obj.MOTOR_R, round(obj.TARGET_RPM * obj.R_MOTOR_SF));
                    obj.body.motor(obj.MOTOR_L, - round(obj.TARGET_RPM));
                    pause(1.5);
                    while(obj.line_within_proximity(obj.body.readReflectance, [1,2,3,4]))
                        pause(0.0001);
                    end
                    reading_ir = obj.body.readReflectance();
                    while(not(obj.line_within_proximity(reading_ir,[1,2,3])))
                        reading_ir= obj.body.readReflectance();
                        pause(0.0001);
                    end
                    obj.body.motor(obj.MOTOR_R, 0);
                    obj.body.motor(obj.MOTOR_L,0);
                    
                    obj.state = States.FollowLineForward;
                    
                    ;
                case States.UnGraspItem % Pd? control for ungrasping an item
                    obj.hasBox = false;
                    obj.findingBox = true;
                    obj.MoveForward(2);
                    obj.state = States.GoBack;
                    ;
                case States.BeFree % Pd? control for
                    ;
                    
                case States.GoBack
                    %                     runtime = 3.0;
                    tic
                    obj.body.motor(obj.MOTOR_R, -round(obj.TARGET_RPM * obj.R_MOTOR_SF));
                    obj.body.motor(obj.MOTOR_L, - obj.TARGET_RPM);
                    
                    while(obj.line_within_proximity(obj.body.readReflectance, [1,2,3,4]))
                        pause(0.0001);
                    end
                    reading_ir = obj.body.readReflectance();
                    while(not(obj.line_within_proximity(reading_ir,[1,2,3])))
                        reading_ir= obj.body.readReflectance();
                        pause(0.0001);
                    end
                    obj.body.motor(obj.MOTOR_R, 0);
                    obj.body.motor(obj.MOTOR_L,0);
                    
                    obj.state = States.TurnAround;
                    ;
                    
            end
        end
        
        function ReturnToZero(obj)
            obj.previousBranch = obj.currentBranch;
            obj.currentBranch = 0;
            %       Turn back into branch zero and keep moving
            obj.MoveForward(2.3);
            %       forward or re orient the robot to choose a new
            %       path
            obj.Turn90DegreeLeft();
            obj.state = States.TurnAround;
        end
        function MoveForward(obj,runtime)
            tic
            obj.body.motor(obj.MOTOR_R, round(obj.TARGET_RPM * obj.R_MOTOR_SF));
            obj.body.motor(obj.MOTOR_L,  obj.TARGET_RPM);
            
            while toc < runtime
            end
            obj.stopMotors();
        end
        function Turn90DegreeLeft(obj)
            runtime = 2.5;
            tic;
            obj.body.motor(obj.MOTOR_R, -round(obj.TARGET_RPM * obj.R_MOTOR_SF));
            obj.body.motor(obj.MOTOR_L, obj.TARGET_RPM);
            while(toc < runtime)
            end
            obj.stopMotors();
        end
        function TurnToBranchX(obj, count)
            disp('Turning');
            currentCount = 0;
            obj.body.motor(obj.MOTOR_R, round(obj.TARGET_RPM * obj.R_MOTOR_SF));
            obj.body.motor(obj.MOTOR_L, round(-obj.TARGET_RPM));
            
            while(currentCount < count)
                values = obj.body.readReflectance();
                if(obj.line_in_sight(values,[2,3])) % this will probably need padding
                    currentCount = currentCount + 1;
                    if(currentCount == count)
                        break;
                    end
                    obj.stopMotors();
                    pause(0.1);
                    obj.body.motor(obj.MOTOR_R, round(obj.TARGET_RPM * obj.R_MOTOR_SF));
                    obj.body.motor(obj.MOTOR_L, round(-obj.TARGET_RPM));
                end
            end
            obj.stopMotors();
        end
        
        function val = normalize_IR_reading(obj, reading)
             val = (reading - obj.min_ir_reading)./ (obj.max_ir_reading - obj.min_ir_reading);
        end
        function [curr_error_M1, curr_error_M2, control_M1, control_M2] = keepTargetSpeed_PD(obj, prev_error_M1, prev_error_M2, prev_control_M1, prev_control_M2)
            
            [Vel_M1, Vel_M2] = obj.body.readEncoderVel();
            
            rpm_M1 = Vel_M1 * (1 / 720) * 60;
            rpm_M2 = Vel_M2 * (1 / 720) * 60;
            
            % Motor 1 (right motor PID values)
            error_M1 = obj.TARGET_RPM - rpm_M1;
            %             error_sum_M1 = error_sum_M1 + error_M1;
            error_delta_M1 = prev_error_M1 - error_M1;
            curr_error_M1 = error_M1;
            
            % Motor 2 (left motor PID values)
            error_M2 = obj.TARGET_RPM - rpm_M2;
            %             error_sum_M2 = error_sum_M2 + error_M2;
            error_delta_M2 = prev_error_M2 - error_M2;
            curr_error_M2 = error_M2;
            
            % Control calculations
            control_M1 = prev_control_M1 + error_M1.*obj.KP_RPM + obj.KD_RPM.*error_delta_M1; %YOU WILL NEED TO EDIT THIS
            control_M2 = prev_control_M2 + error_M2.*obj.KP_RPM + obj.KD_RPM.*error_delta_M2;
        end
        function success = calibrate_IR_Sensor(obj)
            
            obj.body.reflectanceSetup();
            pause(.5)
            
            fprintf("Place ir sensor completelely off the track.\n");
            obj.body.setRGB(255,0,255);
            pause(obj.calibration_time);
            
            [samples,increments] = getSamples(obj);
            if(isequal(samples, [nan, nan, nan, nan]) || increments == 0)
                success = false;
                return;
            end
            mean_min = mean(samples./increments);
            obj.min_ir_reading = [mean_min,mean_min,mean_min,mean_min];
            
            fprintf("Place ir sensor such that the reflective tape covers the entire sensor.\n")
            obj.body.setRGB(255,0,255);
            pause(obj.calibration_time);
            
            [samples, increments] = getSamples(obj);
            if(isequal(samples, [nan, nan, nan, nan]) || increments == 0)
                success = false;
                return;
            end
            mean_max = mean(samples./increments);
            obj.max_ir_reading = [mean_max,mean_max,mean_max,mean_max];
            
            success = true;
            return;
        end
        function [samples, increments] = getSamples(obj)
            tic;
            samples = zeros(1,4);
            blink_red = false;
            increments = 0;
            elapsed_time = 0;
            
            while elapsed_time < obj.calibration_time
                if(mod(elapsed_time, obj.blinking_rate) < 0.1)
                    if(blink_red)
                        obj.body.setRGB(255,69,0);
                        blink_red = false;
                    else
                        obj.body.setRGB(0,69,255);
                        blink_red = true;
                    end
                end
                
                samples = samples + obj.body.readReflectance();
                increments = increments + 1;
                elapsed_time = toc;
            end
        end
        function val = isFork(obj, ir_reading)
            normalized_ir = obj.normalize_IR_reading(ir_reading);
            val = obj.valThresholdG(normalized_ir(1), 1, 0.5) && ...
                obj.valThresholdG(normalized_ir(2), 1, 0.5) && ...
                obj.valThresholdG(normalized_ir(3), 1, 0.5) && ...
                obj.valThresholdG(normalized_ir(4), 1, 0.5);
        end
        function val = valThresholdG(obj, i, j, padding)
            val = i >= (j - padding);
        end
        function val = valThresholdL(obj, i, j, padding)
            val = i <= (j + padding);
        end
        function ScaleFactor = FindSF(obj,inputVal)
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
        
        function [success, block_features] = pickUpAndAnalyzeBlock(obj)

            previousAnalogVal = 0;
            currentAnalogVal = 0;
            success = true;
            block_features = zeros(5,1);
            for i = 10:10:180

                obj.body.servo(4, i);
                currentAnalogVal = obj.body.getAverageData('analog', 5);
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
                if( (currentAnalogVal(obj.SERVO_ANALOG) <= highTresh ) && (currentAnalogVal(obj.SERVO_ANALOG)>= lowTresh ) )
                    try
                        %s 47 i 140, m 35 i 110 , B 23 i 80
                        if (i <= 80 )
                            obj.body.servo(obj.SERVO, i - 23); %s 47 m 35 , B 23
                            size = i - 23;
                            break;
                        elseif(i > 80 && i < 115)
                            obj.body.servo(obj.SERVO, i - 30);
                            size = i - 30;
                            break;
                        else
                            obj.body.servo(obj.SERVO, i - 25);
                            size = i - 25;
                            break;
                        end
                    catch
                        success = false;
                        return;
                    end
                end
                previousAnalogVal = currentAnalogVal(2);
            end
            if(i == 180)
                success = false
                return;
            end
            [r,g,b] = obj.body.rgbRead();
            AnalogData = obj.body.getAverageData('analog',5);
            pause(0.01);

            block_features(5) = obj.normalize_val(size, obj.MAX_SIZE, obj.MIN_SIZE);
            block_features(4:2) = obj.normalize_val([r,g,b]',255,0);
            block_features(1) = obj.normalize_val(AnalogData(obj.HALL_EFFECT), obj.MAX_HALL, obj.MIN_HALL);
        end
        function [curr_error_IR, control_M1, control_M2] = ir_PD_Controller(obj, ir_normalized, prev_error_IR)
            
            % Motor 1 (right motor PID values)
            error_IR = obj.get_IR_error(ir_normalized);
            %             error_sum_M1 = error_sum_M1 + error_M1;
            error_delta_IR = prev_error_IR - error_IR;
            curr_error_IR = error_IR;
            
            % Control calculations
            pd_control_result = error_IR.*obj.KP_IR + obj.KD_IR.*error_delta_IR;
            control_M1 = obj.TARGET_RPM - pd_control_result;
            control_M2 = obj.TARGET_RPM + pd_control_result;
        end
        function error = get_IR_error(obj, reading_ir)
            error = -2*reading_ir(1) - reading_ir(2) + reading_ir(3) + 2*reading_ir(4);
        end
        function [control_M1, control_M2] = setMotorSpeeds(obj, prev_control_M1, prev_control_M2)
            if prev_control_M1 > obj.MAX_RPM
                control_M1 = obj.MAX_RPM;
            elseif prev_control_M1 < (-1 * obj.MAX_RPM)
                control_M1 = -1 * obj.MAX_RPM;
            else
                control_M1 = prev_control_M1;
            end
            
            if prev_control_M2 > obj.MAX_RPM
                control_M2 = obj.MAX_RPM;
            elseif prev_control_M2 < (-1 * obj.MAX_RPM)
                control_M2 = -1 * obj.MAX_RPM;
            else
                control_M2 = prev_control_M2;
            end
        end
        function bool = line_within_proximity(obj, reading, sensors)
            max_avg = mean(obj.max_ir_reading);
            bool = false;
            for i = length(sensors)
                bool = bool || obj.valThresholdG(reading(sensors(i)),max_avg,(3/4).*max_avg);
            end
        end
        function bool = line_in_sight(obj, reading, sensors)
            max_avg = mean(obj.max_ir_reading);
            bool = true;
            for i = length(sensors)
                bool = bool && obj.valThresholdG(reading(sensors(i)),max_avg,(3/4).*max_avg);
            end
        end
        function stopMotors(obj)
            obj.body.motor(obj.MOTOR_L,0);
            obj.body.motor(obj.MOTOR_R,0);
        end
    end
end