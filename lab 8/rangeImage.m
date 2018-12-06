classdef rangeImage < handle
    properties(Constant)
        
        thOffset = -1; % degrees (-1 for robot 11), (2 for 12)
        %atan2(-0.024,0.28);
        sailWidth = 0.127; % m
        
        maxDist = 1.6;
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
    end
    methods
        
    end
    methods(Access = public)
        function obj = rangeImage(robot, numImages, cleanFlag)
            
            numImages = 1;
            
            obj.numPix = 360;
            
            obj.rArray = zeros(1, obj.numPix);
            obj.tArray = zeros(1, obj.numPix);
            obj.xArray = zeros(1, obj.numPix);
            obj.yArray = zeros(1, obj.numPix);
            
            ranges = robot.laser.LatestMessage.Ranges;
            for i=1:360
                obj.rArray(i) = ranges(i);
                obj.tArray(i) = deg2rad(i+obj.thOffset);%mod((i-6),360)*(pi/180);
                obj.xArray(i) = ranges(i)*cos(obj.tArray(i));
                obj.yArray(i) = ranges(i)*sin(obj.tArray(i));
            end
            
            if cleanFlag
                obj.removeBadPoints();
                obj.numPix = length(obj.rArray);
            end
        end
    end
    
    methods(Access = public)
        function [centroidX, centroidY, th] = findLineCandidate(obj, debug,...
                axisInterval, isX, low, high, altLow)
            % find potential sail in range image, ruling out walls because they
            % have a nearest point that is too close, or a nearest point which
            % doesn't change the crosstrack eigenvalue of the point cloud
            
            
            if nargin == 7 && axisInterval
                goodOnes = [];
                for i=1:obj.numPix
                    abs = robToWorld([obj.xArray(i); obj.yArray(i); 0]);
                    if isX
                        restriction = abs(1);
                        alt = abs(2);
                    else
                        restriction = abs(2);
                        alt = abs(1);
                    end
                    
                    if restriction >= low && restriction <= high && alt >= altLow
                        goodOnes(end+1) = i;
                    end
                end
                xArr = obj.xArray(goodOnes);
                yArr = obj.yArray(goodOnes);
            else
                xArr = obj.xArray;
                yArr = obj.yArray;
            end
            
            
            if debug
                close all
                figure
                axis([-1.2 1.2 -1.2 1.2]);
                hold on
                scatter(xArr, yArr);
            end
            
            candidates = [];
            
            for i = 1:obj.numPix
                pointCloudX = [];
                pointCloudY = [];
                x = xArr(i);
                y = yArr(i);
                cloudIndices = [];
                
                for j = 1:obj.numPix
                    
                    x2 = xArr(j);
                    y2 = yArr(j);
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
                        foundAnother = false;
                        
                        for j = 1:obj.numPix
                            if ~ismember(cloudIndices, j)
                                foundAnother = true;
                                xC2 = xArr(j);
                                yC2 = yArr(j);
                                dist = ((centroidX-xC2)^2 + ...
                                    (centroidY-yC2)^2)^0.5;
                                if closest == ~1 || dist < closest
                                    closest = dist;
                                    closestInd = j;
                                end
                            end
                        end
                        
                        if foundAnother
                            pointCloudX(end+1) = xArr(closestInd);
                            pointCloudY(end+1) = yArr(closestInd);
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
                            
                        end
                        
                        if ~foundAnother
                            disp("didn;t find another")
                        end
                        
                        if ~foundAnother || numSailWidths > 0.45 || ...
                                maxLambdaJump > obj.lambdaJumpThreshold
                            
                            
                            %                             disp("lambda jump")
                            %                             disp(maxLambdaJump)
                            %                             disp("numSailWidths")
                            %                             disp(numSailWidths)
                            
                            
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
                        %                                                  disp("diagonal was off")
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
            %             obj.indices = linspace(2,obj.numPix,obj.numPix);
            %             obj.indices = obj.indices(goodOnes);
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