classdef robotTrajectory < handle
    properties(Access = public)
        tArr
        sArr
        xArr
        yArr
        vArr
        wArr
        thetaArr
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
            tArr = zeros(numSamples+1, 1);
            xArr = zeros(numSamples+1, 1);
            yArr = zeros(numSamples+1, 1);
            thetaArr = zeros(numSamples+1, 1);
            vArr = zeros(numSamples+1, 1);
            wArr = zeros(numSamples+1, 1);
            sArr = zeros(numSamples+1, 1);
            
            tArr(1) = 0;
            xArr(1) = x;
            yArr(1) = y;
            thetaArr(1) = theta;
            vArr(1) = 0;
            wArr(1) = 0;
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
                
                tArr(n) = t;
                xArr(n) = x;
                yArr(n) = y;
                thetaArr(n) = theta;
                vArr(n) = V;
                wArr(n) = w;
                sArr(n) = sArr(n-1) + sqrt(dx^2 + dy^2);
            end
            
            obj.tArr = tArr;
            obj.xArr = xArr;
            obj.yArr = yArr;
            obj.thetaArr = thetaArr;
            obj.vArr = vArr;
            obj.wArr = wArr;
            obj.sArr = sArr;
        end
        function vels = getVelsAtTime(obj, t)
            V = interp1(obj.tArr, obj.vArr, t);
            w = interp1(obj.tArr, obj.wArr, t);
            vels = [V ; w];
        end
        function pose = getPoseAtTime(obj, t)
            x = interp1(obj.tArr, obj.xArr, t);
            y = interp1(obj.tArr, obj.yArr, t);
            th = interp1(obj.tArr, obj.thetaArr, t);
            pose = [x ; y ; th];
        end
    end
end