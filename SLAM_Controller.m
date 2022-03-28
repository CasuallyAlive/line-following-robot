classdef SLAM_Controller
    enumeration
        StandBy, FollowLineForward, GraspItem, TurnAround, UnGraspItem, BeFree, Fork;
    end
    properties
        state;
        classifier;
    end
    methods
        function obj = SLAM_Controller(classifier)
            obj.state = StandBy;
            obj.classifier = classifier;
        end
    end
end 