classdef trajectoryFollower < handle
    % ref = figure8ReferenceControl(3,1,0.5);
    % traj = robotTrajectory(1000, 0, [0;0;0], ref);
    % fol = trajectoryFollower(traj);
    properties(Access = public)
        % time constant, lower tau => more feedback
        tau = 1
        % encoder updates come ever 0.02
        sleep = 0.01;
        tdelay = 0.33;
        feedback = true;
        controller
        debug = false;
        sim = false
    end
    
    methods(Access = public)
        function obj = trajectoryFollower(trajectory, isCubic)
            if isCubic
                obj.controller = controller2(trajectory, obj.sleep, ...
                    obj.tdelay, obj.feedback, obj.debug, obj.sim, obj.tau);
            else
                obj.controller = controller(trajectory, obj.sleep, obj.tdelay, ...
                    obj.feedback, obj.debug, obj.sim, obj.tau);
            end
        end
        
        function execute(obj)
            obj.controller.execute();
        end
    end
end