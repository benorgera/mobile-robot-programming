classdef robotTrajectory < handle
    properties(Access = public)
        timeArray
        sArr
        poseArray
        VArray
        wArray
        tf
    end
    methods
        function obj = robotTrajectory(numSamples, s0, p0, ref)
            %  It generates numSamples samples of the time, distance,
            %  velocities, and poses over the specified time interval.
            %  This is done by integration of the velocity signals
            x = p0(1);
            y = p0(2);
            theta = p0(3);
            timeArray = zeros(numSamples+1, 1);
            poseArray = zeros(3, numSamples+1);
            VArray = zeros(numSamples+1, 1);
            wArray = zeros(numSamples+1, 1);
            sArr = zeros(numSamples+1, 1);
            
            timeArray(1) = 0;
            poseArray(1,1) = x;
            poseArray(2,1) = y; 
            poseArray(3,1) = theta;
            VArray(1) = 0;
            wArray(1) = 0;
            sArr(1) = s0;
            
            obj.tf = ref.getTrajectoryDuration(ref);
            dt = obj.tf / numSamples;
            
            for n = 2:(numSamples+1)
                t = dt * (n-1);
                [V,w] = ref.computeControl(t);
                dTheta = w * dt;
                thetaTemp = theta + dTheta / 2.0;
                dx = V * cos(thetaTemp) * dt;
                dy = V * sin(thetaTemp) * dt;
                theta = theta + dTheta;
                x = x + dx;
                y = y + dy;
                
                timeArray(n) = t;
                poseArray(:,n) = [x; y; theta];
                VArray(n) = V;
                wArray(n) = w;
                sArr(n) = sArr(n-1) + sqrt(dx^2 + dy^2);
            end
            
            obj.timeArray = timeArray;
            obj.poseArray = poseArray;
            obj.VArray = VArray;
            obj.wArray = wArray;
            obj.sArr = sArr;
        end
        function V = getVAtTime(obj, t)
            V = interp1(obj.timeArray, obj.VArray, t);
        end
        function w = getwAtTime(obj, t)
            w = interp1(obj.timeArray, obj.wArray, t);
        end
        
        function tf = getTrajectoryDuration(obj)
            tf = obj.tf;
        end
        
        function pose = getPoseAtTime(obj, t)
            x = interp1(obj.timeArray, obj.poseArray(1,:), t);
            y = interp1(obj.timeArray, obj.poseArray(2,:), t);
            th = interp1(obj.timeArray, obj.poseArray(3,:), t);
            pose = [x ; y ; th];
        end
    end
end