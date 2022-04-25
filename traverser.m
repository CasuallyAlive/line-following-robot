%% Setup Connection
clear; clc; close all; % initialization
%% Connect
r = MKR_MotorCarrier;
load("training_examples_raw.mat");
load("Y.mat");
%% Normalize data

training_examples = training_examples_raw;

MAX_SIZE = max(training_examples(5,:));
MIN_SIZE = min(training_examples(5,:));

MAX_HALL = max(training_examples(1,:));
MIN_HALL = min(training_examples(1,:));

CONSTS = [MAX_SIZE,MIN_SIZE,MAX_HALL,MIN_HALL];
hall_effect = training_examples(1,:);

size_b = training_examples(5,:);
training_examples(5,:) = (size_b - min(size_b))./(max(size_b) - min(size_b)); %normalize size

training_examples(2:4,:) = training_examples(2:4,:)./255; %normalize colors
training_examples(1,:) = (training_examples(1,:) - min(hall_effect))./(max(hall_effect) - min(hall_effect));
%% Train Model
classes = unique(Y);

t = templateSVM('Standardize',true,'KernelFunction','polynomial');

mdl_svm  = fitcecoc(training_examples', Y,'Learners',t, 'FitPosterior',true,...
    'ClassNames', classes);
%% Test Model
mdl_svm.predict(training_examples(:,45)')
%% Declare Robot Controller

controller = SLAM_Controller(r);
controller.classifier = mdl_svm;
controller.MAX_SIZE = CONSTS(1);
controller.MIN_SIZE = CONSTS(2);
controller.MAX_HALL = CONSTS(3);
controller.MIN_HALL = CONSTS(4);
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