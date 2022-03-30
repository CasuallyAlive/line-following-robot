%% Setup Connection
clear; clc; close all; % initialization

%% Connect
r = MKR_MotorCarrier; %connect to MKR

%% Main loop for SLAM traversal
controller = SLAM_Controller;

while(1)
    switch(controller.state)
        case SLAM_Controller.FollowLineForward
            break;
        case SLAM_Controller.Fork
            break;
        case SLAM_Controller.GraspItem
            break;
        case SLAM_Controller.TurnAround
            break;
        case SLAM_Controller.UnGraspItem
            break;
        case SLAM_Controller.BeFree
            break;
    end
end