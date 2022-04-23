%% Setup Connection
clear; clc; close all; % initialization
%% Connect
r = MKR_MotorCarrier;
load("training_examples.mat");
load("Y.mat");
%% Train Model
classes = unique(Y);

t = templateSVM('Standardize',true,'KernelFunction','polynomial');

mdl_svm  = fitcecoc(training_examples', Y,'Learners',t, 'FitPosterior',true,...
    'ClassNames', classes);
%% Test Model
mdl_svm.predict(training_examples(:,140)')
%% Declare Robot Controller

controller = SLAM_Controller(MKR_MotorCarrier);
controller.classifier = mdl_svm;
%% Main loop for SLAM traversal

start_robot = false;
while(1)
    if(not(start_robot))
        if(input("Enter 'start' to initiate the map traversal.\n","s") == "start")
            start_robot = true;
        end
        continue;
    end
    controller.do_task();
    if(controller.state == States.StandBy)
        start_robot = false;
    end
end