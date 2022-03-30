classdef SLAM_Controller
    enumeration
        StandBy, FollowLineForward, GraspItem, TurnAround, UnGraspItem, BeFree, Fork;
    end
    properties
        state;
        classifier;
        body;
    end
    methods
        function obj = SLAM_Controller(classifier, body)
            obj.state = StandBy;
            obj.classifier = classifier;
            obj.body = body;
        end
        function [posX, posY] = do_task(obj)
            switch(obj.state)
                case SLAM_Controller.FollowLineForward
                    ;
                case SLAM_Controller.Fork
                    ;
                case SLAM_Controller.GraspItem
                    ;
                case SLAM_Controller.TurnAround
                    ;
                case SLAM_Controller.UnGraspItem
                    ;
                case SLAM_Controller.BeFree
                    ;
            end
        end
    end
end 