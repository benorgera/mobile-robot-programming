classdef rangeImage < handle
    properties(Constant)
        
        thOffset = -2; % degrees
        %atan2(-0.024,0.28); 
        sailWidth = 0.127; % m
        
        maxDist = 1.2;
        minDist = 0.07;
        
        % amount lambda increases by when you add nearest point to cloud
        % (should jump by at least this much if its a sail, a wall won't
        % jump b/c extra point is along same line)
        lambdaJumpThreshold = 10;
        
        pointThreshold = 5;
        lambdaThreshold = 1.3; % cloud crosstrack
        widthThreshold = 0.035;
    end
    
    
    properties(Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix=0;
        indices = [];
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
                    for i=1:length(ranges)
                        n = n + 1;
                        obj.rArray(n) = ranges(i);
                        obj.tArray(n) = deg2rad(i+obj.thOffset);%mod((i-6),360)*(pi/180);
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
        % find potential sail in range image, ruling out walls because they
        % have a nearest point that is too close, or a nearest point which
        % doesn't change the crosstrack eigenvalue of the point cloud
            if debug
                close all
                figure
                axis([-1.2 1.2 -1.2 1.2]);
                hold on
                scatter(obj.xArray, obj.yArray);
            end
            
            candidates = [];
            
            for i = 1:length(obj.rArray)
                pointCloudX = [];
                pointCloudY = [];
                x = obj.xArray(i);
                y = obj.yArray(i);
                cloudIndices = [];
                
                for j = 1:length(obj.rArray)
                    
                    x2 = obj.xArray(j);
                    y2 = obj.yArray(j);
                    dist = ((x-x2)^2 + (y-y2)^2)^0.5;
                    if dist <= obj.sailWidth/2 + obj.widthThreshold
                        % find a included point, do sth
                        pointCloudX(end+1) = x2;
                        pointCloudY(end+1) = y2;
                        cloudIndices(end+1) = j;
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
                        
                        closest = ~1;
                        closestInd = ~1;
                        
                        for j = 1:length(obj.rArray)
                            if ~ismember(cloudIndices, j)
                                xC2 = obj.xArray(j);
                                yC2 = obj.yArray(j);
                                dist = ((centroidX-xC2)^2 + ...
                                    (centroidY-yC2)^2)^0.5;
                                if closest == ~1 || dist < closest
                                    closest = dist;
                                    closestInd = j;
                                end
                            end
                        end
                        
                        pointCloudX(end+1) = obj.xArray(closestInd);
                        pointCloudY(end+1) = obj.yArray(closestInd);
                        centroidX1 = mean(pointCloudX);
                        centroidY1 = mean(pointCloudY);
                        targetCloudX1 = pointCloudX - centroidX1;
                        targetCloudY1 = pointCloudY - centroidY1;
                        Ixx1 = sum(targetCloudX1.^2);
                        Iyy1 = sum(targetCloudY1.^2);
                        Ixy1 = sum(-(targetCloudX1.*targetCloudY1));
                        Inertia1 = [Ixx1 Ixy1;Ixy1 Iyy1] / (pointNum + 1);
                        lambda1 = eig(Inertia1);
                        lambda1 = sqrt(lambda1)*1000.0;
                        
                        maxLambdaJump = abs(lambda(1) - lambda1(1));
                        numSailWidths = closest / obj.sailWidth - 0.5;
                        
                        if numSailWidths > 0.45 || ...
                                maxLambdaJump > obj.lambdaJumpThreshold
                            
                            
                            disp("lambda jump")
                            disp(maxLambdaJump)
                            disp("numSailWidths")
                            disp(numSailWidths)
                            
                            
                            if debug
%                                 disp(diag);
%                                 disp("had centroid")
%                                 disp([centroidX centroidY])
%                                 disp("with number of points")
%                                 disp(pointNum)
                                
                                lx = leftX + centroidX;
                                rx = rightX + centroidX;
                                by = botY + centroidY;
                                ty = topY + centroidY;
                                
                                plot([lx lx rx rx], [by ty by ty], 'bx')
                            end
                            
                            th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                            
                            candidates = [candidates ...
                                [lambda(1); centroidX; centroidY; th; pointNum]];
                        elseif debug
                            disp("nearest point didn't cause lambda jump")
                        end
                    elseif debug
                        %                         disp("diagonal was off")
                    end
                end
                
            end
            
            if isempty(candidates)
                centroidX = 0;
                centroidY = 0;
                th = 0;
            else
                [~, ind] = min(candidates(1,:)); % min lambda 1
                
                data = candidates(:, ind);
                centroidX = data(2);
                centroidY = data(3);
                th = data(4);
                
                disp("lambda was")
                disp(data(1))
                disp("num points")
                disp(data(5))
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
            obj.tArray = obj.tArray(goodOnes);
            obj.indices = linspace(2,obj.numPix,obj.numPix);
            obj.indices = obj.indices(goodOnes);
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