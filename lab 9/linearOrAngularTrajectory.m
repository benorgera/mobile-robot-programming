classdef linearOrAngularTrajectory < handle
    methods(Static = true)
        
        function ref = moveRelDist(dist)
            ref = trapezoidalStepReferenceControl(0.25, ...
                0.7 * robotModel.maxWheelVelocity, abs(dist), ...
                 dist / abs(dist), 0);
        end
        
        function ref = turnRelAngle(theta)
            ref = trapezoidalTurnReference(3.5, ...
                0.8 * robotModel.maxOmega, abs(theta), ...
                 theta / abs(theta), 0);
        end
    end
end