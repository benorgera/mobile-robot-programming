classdef trapezoidalStepReferenceControl < handle
    % ref = trapezoidalStepReferenceControl(0.25, 0.5, 1.5, 1, 0.25)
    % for a 1.5 meter trajectory
    properties
        tramp
        sgn
        tPause
        tf
        amax
        vmax
    end
    methods(Static)
        function dur = getTrajectoryDuration(obj)
            dur = obj.tPause * 2 + obj.tf;
        end
    end
    methods
        function obj = trapezoidalStepReferenceControl ...
                (amax, vmax, dist, sgn, tPause)
            obj.tf = (dist + (vmax^2)/amax)/vmax;
            obj.tramp = vmax / amax;
            obj.sgn = sgn;
            obj.tPause = tPause;
            obj.amax = amax;
            obj.vmax = vmax;
        end
        
        function [uref, w] = computeControl(obj, timeNow)
            % Construct a trapezoidal profile. It will not start until
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards
            if timeNow < obj.tPause || timeNow > obj.tPause + obj.tf
                uref = 0;
            else
                t = timeNow - obj.tPause;
                if t < 0
                    uref = 0;
                elseif t < obj.tramp
                    uref = obj.amax * t;
                elseif (obj.tf - t) < obj.tramp
                    uref = obj.amax*(obj.tf-t);
                elseif (t> obj.tramp && t < (obj.tf-obj.tramp))
                    uref = obj.vmax;
                else
                    uref = 0;
                end
                if uref<0
                    uref = 0;
                end
                uref = obj.sgn*uref;
            end
            w = 0.0;
        end
    end
end