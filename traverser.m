%% Setup Connection
clear; clc; close all; % initialization
%% Connect
controller = SLAM_Controller(MKR_MotorCarrier);
%% Main loop for SLAM traversal

start_robot = false;
while(1)
    if(not(start_robot))
        if(input("Enter 'start' to initiate the map traversal.\n","s") == "start")
            start_robot = true;
        end
        continue;
    end
%     if(start_robot && input("\n", "s") == "abort")
%         fprintf("Traversal was aborted!");
%         break;
%     end
    controller.do_task();
    if(controller.state == States.StandBy)
        start_robot = false;
    end
end