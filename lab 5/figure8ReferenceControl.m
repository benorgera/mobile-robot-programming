classdef figure8ReferenceControl < handle
    % ref = figure8ReferenceControl(3,1,0.5);
    properties
        ks
        kv
        tPause
        v = 0.2;
        sf = 1;
        tf
        kth
        kk = 15.1084;
        Tf
    end
    methods(Static)
        function dur = getTrajectoryDuration(obj)
            dur = obj.tPause * 2 + ...
                obj.Tf;
        end
    end
    methods
        function obj = figure8ReferenceControl(ks,kv,tPause)
            obj.ks = ks;
            obj.kv = kv;
            obj.tPause = tPause;
            obj.tf = obj.sf / obj.v;
            obj.Tf = ks * obj.tf;
            obj.kth = 2 * pi / obj.sf;
        end
        
        function [V, w] = computeControl(obj, timeNow)
            % Construct a figure 8 trajectory. It will not start until
            % tPause has elapsed and it will stay at zero for tPause
            % afterwards. Kv scales velocity up when > 1 and Ks scales
            % the size of the curve itself up.
            
            T = timeNow - obj.tPause;
            if T < 0 || T > obj.Tf
                V = 0;
                w = 0;
            else
                t = T / obj.ks;
                s = obj.v * t;
                K = obj.kk / obj.ks...
                    * sin(obj.kth * s);
                V = obj.kv * obj.v;
                w = K * V;
            end
        end
    end
end