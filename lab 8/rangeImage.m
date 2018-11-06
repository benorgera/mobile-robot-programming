classdef rangeImage < handle
    properties(Constant)
        
        thOffset = atan2(-0.024,0.28); %
        sailWidth = 0.1; % m
        
        maxDist = 1.0;
        minDist = 0.07;
        
        
        pointThreshold = 7;
        lambdaThreshold = 1.3;
        widthThreshold = 0.012;
    end
    
    
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix=0;
        indices = [];
        th = [];
    end
    methods
        
    end
    methods(Access = public)
        function obj = rangeImage(robot, numImages, cleanFlag)
            
            obj.rArray = zeros(1, numImages * 360);
            obj.tArray = zeros(1, numImages * 360);
            obj.xArray = zeros(1, numImages * 360);
            obj.yArray = zeros(1, numImages * 360);
            
            if(nargin == 3)
                n=0;
                for j = 1:numImages
                    ranges = robot.laser.LatestMessage.Ranges;
                    disp(length(ranges))
                    for i=1:length(ranges)
                        n = n + 1;
                        obj.rArray(n) = ranges(i);
                        obj.tArray(n) = deg2rad(i-6);%mod((i-6),360)*(pi/180);
                        obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                        obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                    end
                    pause(0.2)
                end
                obj.numPix = n;
                if cleanFlag
                    obj.removeBadPoints();
                end
            end
        end
    end
    
    methods(Access = public)
        function [centroidX, centroidY, th] = findLineCandidate(obj, debug)
            
            if debug
                close all
                figure
                hold on
                scatter(obj.xArray, obj.yArray);
            end
            
            candidates = [];
            
            for i = 1:length(obj.rArray)
                pointCloudX = [];
                pointCloudY = [];
                x = obj.xArray(i);
                y = obj.yArray(i);
                
                for j = 1:length(obj.rArray)
                    
                    x2 = obj.xArray(j);
                    y2 = obj.yArray(j);
                    dist = ((x-x2)^2 + (y-y2)^2)^0.5;
                    if dist <= obj.sailWidth/2 + obj.widthThreshold
                        % find a included point, do sth
                        pointCloudX(end+1) = x2;
                        pointCloudY(end+1) = y2;
                    end
                    
                end
                pointNum = length(pointCloudX);
                
                if ((pointNum >= obj.pointThreshold))
                    centroidX = mean(pointCloudX);
                    centroidY = mean(pointCloudY);
                    targetCloudX = pointCloudX - centroidX;
                    targetCloudY = pointCloudY - centroidY;
                    Ixx = sum(targetCloudX.^2);
                    Iyy = sum(targetCloudY.^2);
                    Ixy = sum(-(targetCloudX.*targetCloudY));
                    Inertia = [Ixx Ixy;Ixy Iyy] / pointNum;
                    lambda = eig(Inertia);
                    lambda = sqrt(lambda)*1000.0;
                    
                    leftX = min(targetCloudX);
                    rightX = max(targetCloudX);
                    topY = max(targetCloudY);
                    botY = min(targetCloudY);
                    diag = sqrt((rightX - leftX)^2 + (botY - topY)^2);
                    
                    if abs(diag - obj.sailWidth) <= obj.widthThreshold
                        
                        if debug
                            disp(diag);
                            disp("had centroid")
                            disp([centroidX centroidY])
                            disp("with number of points")
                            disp(pointNum)
                            
                            lx = leftX + centroidX;
                            rx = rightX + centroidX;
                            by = botY + centroidY;
                            ty = topY + centroidY;
                            
                            plot([lx lx rx rx], [by ty by ty], 'bx') 
                        end
                        
                        th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        
                        candidates = [candidates ...
                            [lambda(1); centroidX; centroidY; th]];
                    end
                end
                
            end
            
            if isempty(candidates)
                centroidX = 0;
                centroidY = 0;
                th = 0;
            else
                [~, ind] = min(candidates(1,:));
                
                data = candidates(:, ind);
                centroidX = data(2);
                centroidY = data(3);
                th = data(4);
            end
        end
        function num = numPixels(obj)
            num = obj.numPix;
        end
        function removeBadPoints(obj)
            
            goodOnes = obj.rArray > obj.minDist & obj.rArray < obj.maxDist;
            obj.rArray = obj.rArray(goodOnes);
            obj.xArray = obj.xArray(goodOnes);
            obj.yArray = obj.yArray(goodOnes);
            obj.indices = linspace(2,obj.numPix,obj.numPix);
            obj.indices = obj.indices(goodOnes);
            % Compute the angles of surviving points
            obj.th = deg2rad(obj.indices-1) - obj.thOffset;
        end
    end
    
    
    methods(Static)
        % convert the rangeImage index and range to
        % x,y th
        function [x,y,th] = irToXY(i,r)
            th = deg2rad((i-1));
            th = th-obj.thOffset;
            x =  r*cos(th);
            y =  r*sin(th);
            
            
        end
        
        
    end
end