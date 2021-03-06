classdef trapezoidalTurnReference < handle
    % ref = trapezoidalStepReferenceControl(0.25, 0.5, 1.5, 1, 0.25)
    % for a 1.5 meter trajectory
    properties
        tramp
        sgn
        tPause
        tf
        tplateau
        amax
        wmax
    end
    methods(Static)
        function dur = getTrajectoryDuration(obj)
            dur = obj.tPause * 2 + obj.tf;
        end
    end
    methods
        function obj = trapezoidalTurnReference(amax, wmax, ...
                theta, sgn, tPause)
            obj.tramp = wmax / amax;
            if obj.tramp * wmax > theta
                wmax = theta / obj.tramp;
                amax = wmax / obj.tramp;
            end
            obj.tf = obj.tramp + theta / wmax;
            obj.sgn = sgn;
            obj.tPause = tPause;
            obj.amax = amax;
            obj.wmax = wmax;
        end
        
        function [v, w] = computeControl(obj, timeNow)
            % Construct a trapezoidal profile. It will not start until
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards
            if timeNow < obj.tPause || timeNow > obj.tPause + obj.tf
                w = 0;
            else
                t = timeNow - obj.tPause;
                if t < 0
                    w = 0;
                elseif t < obj.tramp
                    w = obj.amax * t;
                elseif (obj.tf - t) < obj.tramp % ramping down
                    w = obj.amax*(obj.tf-t);
                elseif (t> obj.tramp && t < (obj.tf-obj.tramp))
                    w = obj.wmax;
                else
                    w = 0;
                end
                if w<0
                    w = 0;
                end
                w = obj.sgn*w;
            end
            v = 0.0;
        end
    end
end