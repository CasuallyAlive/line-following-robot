classdef SLAM_Controller
    enumeration
        StandBy, Calibration, FollowLineForward, GraspItem, TurnAround, UnGraspItem, BeFree, Fork;
    end
    properties
        state;
        classifier;
        body;
        destinations_visited; % nodes in the track graph
        forks_visited;
        forks_completely_traversed;
        track_graph;
        calibration_time;
        max_ir_reading;
        min_ir_reading;
        blinking_rate;
        is_calibrated;
    end
    methods
        function obj = SLAM_Controller(classifier, body)
            obj.state = StandBy;
            obj.classifier = classifier;
            obj.body = body;
            obj.max_ir_reading = [nan, nan, nan, nan];
            obj.min_ir_reading = [nan, nan, nan, nan];
            obj.calibration_time = 5;
            obj.blinking_rate = 0.125;
            obj.is_calibrated = false;
        end
        function [posX, posY] = do_task(obj)
            switch(obj.state)
                case SLAM_Controller.StandBy
                    if(obj.is_calibrated)
                        obj.state = FollowLineForward;
                    else
                        obj.state = calibrate_IR_Sensor;
                        obj.body.setRGB(255,0,0);
                    end
                    posX = 0;
                    posY = 0;
                    return;
                case SLAM_Controller.Calibration
                    success = false;
                    while (not(success))
                        success = calibrate_IR_Sensor();
                    end
                    printf("Success! The IR sensor has been calibrated.");
                    obj.body.setRGB(0,255,0)
                    obj.state = StandBy;
                    obj.is_calibrated = true;

                    posX = 0;
                    posY = 0;
                    return;
                case SLAM_Controller.FollowLineForward % PID control for IR sensor and speed
                    
                    ;
                case SLAM_Controller.Fork % Follow a path, tie break tbd, if the path leads to a destination node that has not been visited.
                    ;
                case SLAM_Controller.GraspItem % Pd? control for grasping the object
                    ;
                case SLAM_Controller.TurnAround % Pd? control for rotating the robot a complete 180
                    ;
                case SLAM_Controller.UnGraspItem % Pd? control for ungrasping an item
                    ;
                case SLAM_Controller.BeFree % Pd? control for 
                    ;
            end

            function success = calibrate_IR_Sensor(obj)
                fprintf("Place ir sensor completelely off the track.\n")
                obj.body.setRGB(255,0,255);
                pause(obj.calibration_rate);
                
                [samples,increments] = getSamples();
                if(isequal(samples, [nan, nan, nan, nan]) || increments == 0)
                    success = false;
                    return;
                end
                obj.min_ir_reading = samples./increments;
                
                fprintf("Place ir sensor such that the reflective tape covers the entire sensor.\n")
                obj.body.setRGB(255,0,255);
                pause(obj.calibration_rate);
                
                [samples, increments] = getSamples();
                if(isequal(samples, [nan, nan, nan, nan]) || increments == 0)
                    success = false;
                    return;
                end
                obj.max_ir_reading = samples./increments;
                success = true;
                return;
            end
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
            return;
        end
    end
end 