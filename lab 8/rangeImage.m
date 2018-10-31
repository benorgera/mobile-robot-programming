classdef rangeImage < handle
    properties(Constant)

        thOffset = atan2(-0.024,0.28);
        sailWidth = 0.127; % m 
        maxDist = 4.0;
        maxUsefulRange = 2.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
        threshold = 0.01;
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
        function obj = rangeImage(ranges,skip,cleanFlag)
            
            if(nargin == 3)
                n=0;
                for i=1:skip:length(ranges)
                    n = n + 1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = deg2rad(i-6);%mod((i-6),360)*(pi/180);
                    obj.xArray(n) = ranges(i)*cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i)*sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag 
                    obj.removeBadPoints(); 
                end
            end
        end
    end
    
    methods(Access = public)
        function [centroidX, centroidY, th] = findLineCandidate(obj)

            centroidX = 0;
            centroidY = 0;
            th = 0;
            
            for i = 1 : length(obj.rArray)
                
                
                pointCloudX = [];
                pointCloudY = [];
                x = obj.xArray(i); 
                y = obj.yArray(i);
                
                for j = 1:length(obj.rArray)
                    
                    x2 = obj.xArray(j); 
                    y2 = obj.yArray(j);
                    dist = ((x-x2)^2 + (y-y2)^2)^0.5;
                    if dist <= (obj.sailWidth/2 + obj.threshold)
                        % find a included point, do sth
                       pointCloudX(end+1) = x2;
                       pointCloudY(end+1) = y2;
                    end
                   
                end
                pointNum = length(pointCloudX);
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
                
                
                
                if ((pointNum >= 7) && (lambda(1) < 1.3) )
                    disp("yep!");
                    leftX = min(targetCloudX);
                    rightX = max(targetCloudX);
                    topY = max(targetCloudY);
                    botY = min(targetCloudX);
                    diag = sqrt((rightX - leftX)^2 + (botY - topY)^2);
                    
                    if diag < obj.sailWidth
                        th = atan2(2*Ixy,Iyy-Ixx)/2.0;
                        disp("found one !!!!");
                        disp(centroidX);
                        disp(centroidY);
                        disp(th);
                        
                        break
                    end
                else
                    disp("can't find the point")
                    centroidX = 0;
                    centroidY = 0;
                    th = 0;
                end
                
               
                
            end
            
            
        end
        function num = numPixels(obj)
            num = obj.numPix;
        end
        function removeBadPoints(obj)
       
            goodOnes = obj.rArray > 0.06 & obj.rArray < 4.0;
            obj.rArray = obj.rArray(goodOnes);
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