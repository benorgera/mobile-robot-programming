classdef lineMapLocalizer < handle
    %mapLocalizer A class to match a range scan against a map in
    % order to find the true location of the range scan relative to
    % the map.
    
    properties(Constant)
        maxErr = 0.1; % 5 cm
        minPts = 5; % min # of points that must match
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
        lines_p1 = [];
        lines_p2 = [];
        gain = 0.3;
        errThresh = 0.01;
        gradThresh = 0.0005;
        debug;
        bodyPts;
    end
    methods(Access = public)
        
        function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh, debug)
            % create a lineMapLocalizer
            obj.lines_p1 = lines_p1;
            obj.lines_p2 = lines_p2;
            obj.gain = gain;
            obj.errThresh = errThresh;
            obj.gradThresh = gradThresh;
            obj.debug = debug;
            obj.bodyPts = robotModel.bodyGraph();
        end
        
        function ro2 = closestSquaredDistanceToLines(obj,pi)
            % Find the squared shortest distance from pi to any line
            % segment in the supplied list of line segments.
            % pi is an array of 2d points
            % throw away homogenous flag
            pi = pi(1:2,:);
            r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
            for i = 1:size(obj.lines_p1,2)
                [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
                    obj.lines_p1(:,i),obj.lines_p2(:,i));
            end
            ro2 = min(r2Array,[],1);
        end
        
        function ids = throwOutliers(obj,pose,ptsInModelFrame)
            % Find ids of outliers in a scan.
            worldPts = pose.bToA()*ptsInModelFrame;
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            ids = find(r2 >lineMapLocalizer.maxErr^2);
        end
        
        function avgErr2 = fitError(obj,pose,ptsInModelFrame)
            % Find the variance of perpendicular distances of
            % all points to all lines
            
            % transform the points
            worldPts = pose.bToA()*ptsInModelFrame;
            
            r2 = obj.closestSquaredDistanceToLines(worldPts);
            r2(r2 == Inf) = [];
            err2 = sum(r2);
            num = length(r2);
            if(num >= lineMapLocalizer.minPts)
                avgErr2 = err2/num;
            else
                % not enough points to make a guess
                avgErr2 = inf;
            end
        end
        
        
        function [err2_Plus0,J] = getJacobian(obj,poseIn,modelPts)
            % Computes the gradient of the error function
            
            err2_Plus0 = fitError(obj,poseIn,modelPts);
            
            eps = 1e-9;
            dpX = [eps ; 0.0 ; 0.0];
            dpY = circshift(dpX, 1);
            dpT = circshift(dpX, 2);
            newPoseX = pose(poseIn.getPoseVec()+dpX);
            newPoseY = pose(poseIn.getPoseVec()+dpY);
            newPoseT = pose(poseIn.getPoseVec()+dpT);
            
            err2_PlusX = fitError(obj, newPoseX, modelPts);
            err2_PlusY = fitError(obj, newPoseY, modelPts);
            err2_PlusT = fitError(obj, newPoseT, modelPts);
            
            J = [(err2_PlusX - err2_Plus0)/eps; ...
                (err2_PlusY - err2_Plus0)/eps; ...
                (err2_PlusT - err2_Plus0)/eps];
        end
        
        function [success, outPose] = refinePose(obj,inPose, ...
                modelPts,maxIters)
            % refine robot pose in world (inPose) based on lidar
            % registration. Terminates if maxIters iterations is
            % exceeded or if insufficient points match the lines.
            % Even if the minimum is not found, outPose will contain
            % any changes that reduced the fit error. Pose changes that
            % increase fit error are not included and termination
            % occurs thereafter.
            
            curPose = inPose;
            ids = obj.throwOutliers(inPose, modelPts);
            modelPts(:, ids) = [];
            dim = size(modelPts);
            
            if dim(2) < lineMapLocalizer.minPts
                success = false;
                outPose = inPose;
                return
            end
            
            [err, grad] = obj.getJacobian(inPose, modelPts);
            prevErr = err;
            prevPose = inPose;
            gradMag = norm(grad);
            i = 1;
            
            while err > obj.errThresh || gradMag > obj.gradThresh
                
                if i >= maxIters
                    disp("too many iterations")
                    break;
                end
                
                curPose = pose(curPose.getPoseVec() + -obj.gain * grad);
                [err, grad] = obj.getJacobian(curPose,modelPts);
                gradMag = norm(grad);
                
                if err > prevErr
                    curPose = prevPose;
                    disp("gradient ascent")
                    break;
                end
                prevErr = err;
                prevPose = curPose;
                i = i + 1;
            end
            
            if obj.debug
                hold off
                plot(obj.lines_p1, obj.lines_p2)
                hold on
                transform = curPose.bToA();
                worldPts = transform*modelPts;
                scatter(worldPts(1, :), worldPts(2, :), 'bx')
                
                worldBodyPts = transform*obj.bodyPts;
                plot(worldBodyPts(1,:),worldBodyPts(2,:),'k');
            end
            
            
            success = true;
            outPose = curPose;
        end
        
    end
    
    
end