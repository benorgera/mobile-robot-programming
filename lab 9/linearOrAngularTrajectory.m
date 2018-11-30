classdef linearOrAngularTrajectory < handle
    methods(Static = true)
        
        function ref = moveRelDist(dist)
            ref = trapezoidalStepReferenceControl(0.25, ...
                0.6 * robotModel.maxWheelVelocity, abs(dist), ...
                 dist / abs(dist), 0);
        end
        
        function ref = turnRelAngle(theta)
            ref = trapezoidalTurnReference(3, ...
                0.25 * robotModel.maxOmega, abs(theta * 0.93), ...
                 theta / abs(theta), 0);
        end
    end
end