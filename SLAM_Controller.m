classdef SLAM_Controller < handle
    properties
        state; % the state of the robot.
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
%         is_calibrated;
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
        
        R_MOTOR_SF = 1.2;

        MOTOR_L = 4;
        MOTOR_R = 3;
    end
    methods
        function obj = SLAM_Controller(body)
            obj.state = States.StandBy;
            %obj.classifier = classifier;
            obj.body = body;
            obj.max_ir_reading = [nan, nan, nan, nan];
            obj.min_ir_reading = [nan, nan, nan, nan];
            obj.calibration_time = 5;
            obj.blinking_rate = 0.125;
            obj.is_calibrated = false;
        end
        function [posX, posY] = do_task(obj)
            switch(obj.state)
                case States.StandBy
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
 
                    ir_reading = obj.body.readReflectance();
                    pause(0.1)

                    control_M1 = 0; control_M2 = 0;
                    error_IR = 0; prev_error_IR = 0;
                    tic;
                    while not(obj.isFork(ir_reading))
%                         [curr_error_M1, curr_error_M2, control_M1, control_M2] = obj.keepTargetSpeed_PD(prev_error_M1, prev_error_M2, prev_control_M1, prev_control_M2);
                        [error_IR, control_M1, control_M2] = obj.ir_PD_Controller(obj.normalize_IR_reading(ir_reading),prev_error_IR);
                        
                        [control_M1, control_M2] = obj.setMotorSpeeds(control_M1, control_M2);

                        obj.body.motor(obj.MOTOR_R, round(control_M1*obj.R_MOTOR_SF));
                        obj.body.motor(obj.MOTOR_L, round(control_M2));
                        prev_error_IR = error_IR;
                        ir_reading = obj.body.readReflectance();
                    end
                    obj.body.motor(obj.MOTOR_R, 0);
                    obj.body.motor(obj.MOTOR_L, 0);
                    
                    pause(0.01);

                    elapsed_time = toc;
                    obj.state = States.TurnAround;
                    ;
                case States.Fork % Follow a path, tie break tbd, if the path leads to a destination node that has not been visited.
                    ;
                case States.GraspItem % Pd? control for grasping the object
                    ;
                case States.TurnAround % Pd? control for rotating the robot a complete 180
                    tic;
                    obj.body.motor(obj.MOTOR_R, round(obj.TARGET_RPM * obj.R_MOTOR_SF));
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
                    obj.state = States.FollowLineForward;
                    ;
                case States.UnGraspItem % Pd? control for ungrasping an item
                    ;
                case States.BeFree % Pd? control for 
                    ;
            end
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
            val = obj.valThreshold(normalized_ir(1), 1, 0.5) && ...
                obj.valThreshold(normalized_ir(2), 1, 0.5) && ...
                obj.valThreshold(normalized_ir(3), 1, 0.5) && ...
                obj.valThreshold(normalized_ir(4), 1, 0.5);
        end
        function val = valThreshold(obj, i, j, padding)
            val = i >= (j - padding); 
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
            min_avg = mean(obj.min_ir_reading);
            bool = false;
            for i = length(sensors)
                bool = bool || (reading(sensors(i)) >= min_avg + 200);
            end
        end
    end
end 