%% Setup Connection
clear; clc; close all; % initialization

%% Connect
r = MKR_MotorCarrier; %connect to MKR

%% Main loop for SLAM traversal
controller = SLAM_Controller(null, r);

start_robot = false;
while(1)
    if(not(start_robot))
        if(input("Enter 'start' to initiate the map traversal.\n","s") == "start")
            start_robot = true;
        end
        continue;
    end
    if(start_robot && input("\n", "s") == "abort")
        fprintf("Traversal was aborted!");
        break;
    end
    x,y = controller.do_task();
    if(controller.state == controller.StandBy)
        start_robot = false;
    end
end