classdef trajectoryFollower < handle
    % ref = figure8ReferenceControl(3,1,0.5);
    % traj = robotTrajectory(1000, 0, [0;0;0], ref);
    % fol = trajectoryFollower(traj);
    properties(Access = public)
        % time constant, lower tau => more feedback
        tau = 0.8;
        wMaxFeedback = 0.5;
        justFeedbackPeriod = 1.5;%1.5
        % encoder updates come ever 0.02
        sleep = 0.03;
        % less tdelay moves the robot trajectory right (more delayed)
        tdelay = 0.15;
        feedback = true;
        controller
        plotData = true;
        debug = false;
        sim = false;
    end
    
    methods(Access = public)
        function obj = trajectoryFollower(trajectory, feedback, ...
                justFeedbackPeriod, plotData)
            if nargin >= 2
                obj.feedback = feedback;
            end
            if nargin >= 3
                obj.justFeedbackPeriod = justFeedbackPeriod;
            end
            if nargin == 4
                obj.plotData = plotData;
            end
            
            obj.controller = controller(trajectory, obj.sleep, ...
                obj.tdelay, obj.feedback, obj.debug, obj.plotData, ...
                obj.sim, obj.tau, obj.wMaxFeedback, obj.justFeedbackPeriod);
        end
        
        function execute(obj, initialized, mapPlot)
            if nargin == 3
                obj.controller.execute(initialized, mapPlot);
            else
                obj.controller.execute(initialized);
            end
            
        end
    end
end