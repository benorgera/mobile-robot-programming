classdef trajectoryFollower < handle
    % ref = figure8ReferenceControl(3,1,0.5);
    % traj = robotTrajectory(1000, 0, [0;0;0], ref);
    % fol = trajectoryFollower(traj);
    properties(Access = public)
        % time constant, lower tau => more feedback
        tau = 1
        wMaxFeedback = 1;
        % encoder updates come ever 0.02
        sleep = 0.02;
        tdelay = 0.33;
        feedback = false;
        controller
        plotData = true;
        debug = false;
        sim = true;
    end
    
    methods(Access = public)
        function obj = trajectoryFollower(trajectory)
            obj.controller = controller(trajectory, obj.sleep, ...
                obj.tdelay, obj.feedback, obj.debug, obj.plotData, ...
                obj.sim, obj.tau, obj.wMaxFeedback);
        end
        
        function execute(obj, initialized)
            obj.controller.execute(initialized);
        end
    end
end