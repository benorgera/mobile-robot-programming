classdef controller < handle
    % instantiated from trajectoryFollower
    % only difference from original controller is that it uses
    % cubicSpiralTrajectory, which has pose as 1 array instead of 3 (and
    % some different syntax for getting vels)
    properties
        traj
        tf
        tau
        wMaxFeedback
        justFeedbackPeriod
        feedback
        sleep
        tdelay
        tArr
        poseArr
        vArr
        wArr
        poseErrArr
        debug % should it plot feedback vectors
        plotData % should it plot things other than position
        sim
        logLength = 1000;
        logIndex = 1;
        maxNgFeedback = 0.3;
    end
    methods
        function obj = controller(traj, sleep, tdelay, feedback, debug, ...
                plotData, sim, tau, wMaxFeedback, justFeedbackPeriod)
            %  follows a reference trajectory for feed forward control,
            %  with proportional feedback
            obj.tf = traj.getTrajectoryDuration();
            obj.traj = traj;
            obj.feedback = feedback;
            obj.sleep = sleep;
            obj.tdelay = tdelay;
            obj.debug = debug;
            obj.plotData = plotData;
            obj.sim = sim;
            obj.tau = tau;
            obj.wMaxFeedback = wMaxFeedback;
            obj.justFeedbackPeriod = justFeedbackPeriod;
            
            if obj.plotData
                obj.poseErrArr = zeros(3, obj.logLength);
                obj.tArr = zeros(obj.logLength, 1);
                obj.vArr = zeros(obj.logLength, 1);
                obj.wArr = zeros(obj.logLength, 1);
                obj.poseArr = zeros(2, obj.logLength); % theta not needed to show position
            else
                obj.poseArr = zeros(3, obj.logLength);
            end
        end
        
        function execute(obj, initialized, mapPlot)
            if nargin == 3 && mapPlot
                global map
                mapPlot = true;
                mapFig = figure;
            else
                mapPlot = false;
            end
            
            if obj.plotData
                erW = [];
                erR = [];
                f = figure;
                p = plot(obj.poseArr(1,:), obj.poseArr(2,:), 'b-');
                axis([ min(obj.traj.poseArray(1,:)) - 0.1 ...
                    max(obj.traj.poseArray(1,:)) + 0.1 ...
                    min(obj.traj.poseArray(2,:)) - 0.1 ...
                    max(obj.traj.poseArray(2,:)) + 0.1]);
                hold on
                plot(obj.traj.poseArray(1,:), obj.traj.poseArray(2,:), 'r-');
                hold on
            end
            
            % communicate with encoder callbacks
            global robot;
            global robotPose;
            
            if ~initialized
                
                name = 'ribit';
                if (obj.sim)
                    name = 'sim';
                end
                
                if (obj.feedback)
                    disp('Running with feedback')
                else
                    disp('Running without feedback')
                end
                
                robotPose = [0; 0; 0; 0; 0];
                robot = raspbot(name);
                robot.stop();
                pause(2)
            end
            
            if length(robotPose) ~= 5
                robotPose = [0; 0; 0; 0; 0];
            end
            
            % previous trajectory removes this so you need to add it back
            robot.encoders.NewMessageFcn=@encoderEventListener;
            
            % initialize feedback P controller coefficients
            kx = 1/obj.tau;
            ktheta = kx;
            
            tic
            while (true)
                t = toc;
                if t >= obj.tf + obj.justFeedbackPeriod
                    break
                end
                
                curPose = robotPose;
                rx = curPose(1);
                ry = curPose(2);
                rtheta = curPose(3);
                
                V = 0.0;
                w = 0.0;
                refV = obj.traj.getVAtTime(max(0, min(t+obj.tdelay, obj.tf)));
                % last 1 second will just be feedback
                if t < obj.tf
                    V = refV;
                    w = obj.traj.getwAtTime(max(0, min(t+obj.tdelay, obj.tf)));
                end
                
                % need actual pose to compute error
                refPose = obj.traj.getPoseAtTime(max(0, min(t+obj.tdelay, obj.tf)));
                
                % this constant is dependent on vel
                ky = 0.0;
                if (refV ~= 0)
                    ky = 2 / (abs(refV)*obj.tau^2);
                    %                     disp('ky')
                    %                     disp(ky)
                end
                
                % error vector in world frame
                er = refPose - [rx; ry; rtheta];
                erTheta = atan2(sin(er(3)), cos(er(3)));
                
                s = sin(rtheta);
                c = cos(rtheta);
                rthetaNorm = atan2(s, c);
                
                erVectorRobot = [0 ; 0];
                if (rtheta ~= 0 && rthetaNorm ~= 0)
                    erVectorRobot = [c -s; s c] \ [er(1) ; er(2)];
                    % rotate to robot coords (A\b is same as A^-1 b)
                end
                
                % compute feedback, accounting for angular error
                uv = kx * erVectorRobot(1);
                uw = erTheta * ktheta;
                
                if (t < obj.tf) % can't correct crosstrack error after ref
                    uw = uw + ky * erVectorRobot(2);
                end
                
                if abs(uw) > obj.maxNgFeedback
                    uw = obj.maxNgFeedback * sign(uw);
                end
                
                if (obj.feedback)
                    
                    %                     disp('raw feedback')
                    %                     disp([uv uw])
                    V = V + uv;
                    w = w + uw;
                end
                
                if (obj.logIndex <= obj.logLength)
                    if (obj.debug)
                        figure(f)
                        delete(erW);
                        delete(erR);
                        erW = quiver(rx, ry, er(1), er(2), 0);
                        erR = quiver(0, 0, erVectorRobot(1), erVectorRobot(2));
                    end
                    
                    if obj.plotData
                        
                        obj.poseArr(1, obj.logIndex) = rx;
                        obj.poseArr(2, obj.logIndex) = ry;
                        set(p,'Xdata',obj.poseArr(1, 1:obj.logIndex))
                        set(p,'Ydata',obj.poseArr(2, 1:obj.logIndex))
                        
                        
                        obj.poseArr(3, obj.logIndex) = rtheta;
                        obj.tArr(obj.logIndex) = t;
                        obj.vArr(obj.logIndex) = robotPose(4);
                        obj.wArr(obj.logIndex) = robotPose(5);
                        obj.poseErrArr(:, obj.logIndex) = ...
                            [ erVectorRobot; erTheta ];
                    end
                    obj.logIndex = obj.logIndex + 1;
                end
                
                if mapPlot
                    figure(mapFig)
                    
                    modelPts = map.lastModelPts;
                    curPose = robotPose(1:3);
                    
                    hold off
                    plot(map.lines_p1, map.lines_p2)
                    hold on
                    transform = pose(curPose).bToA();
                    worldPts = transform*modelPts;
                    scatter(worldPts(1, :), worldPts(2, :), 'bx')
                    
                    worldBodyPts = transform*map.bodyPts;
                    plot(worldBodyPts(1,:),worldBodyPts(2,:),'k');
                end
                
                [vl, vr] = robotModel.VwTovlvr(V, w);
                if (isnan(vl) || isnan(vr))
                    disp('pose')
                    disp(refPose)
                    disp('ribit')
                    disp(rx)
                    disp(ry)
                    disp(rtheta)
                    disp('erW')
                    disp(er)
                    disp('erR')
                    disp(erVectorRobot)
                end
                
                [vl, vr] = robotModel.VwTovlvr(V, w);
                robot.sendVelocity(vl, vr)
                pause(obj.sleep)
            end
            robot.stop()
            robot.encoders.NewMessageFcn=[];
            
            if obj.plotData
                len = obj.logIndex - 1;
                
                figure
                plot(obj.tArr(1:len), obj.poseArr(1, 1:len))
                hold on
                plot(obj.tArr(1:len),obj.poseArr(2, 1:len))
                hold on
                plot(obj.tArr(1:len), obj.poseArr(3, 1:len))
                hold on
                plot(obj.traj.timeArray, obj.traj.poseArray(1,:))
                hold on
                plot(obj.traj.timeArray, obj.traj.poseArray(2,:))
                hold on
                plot(obj.traj.timeArray, obj.traj.poseArray(3,:))
                hold on
                legend({'xRob', 'yRob', 'thetaRob', 'xRef', 'yRef', 'thetaRef'});
                
                figure
                plot(obj.tArr(1:len), obj.poseErrArr(1, 1:len))
                hold on
                plot(obj.tArr(1:len), obj.poseErrArr(2, 1:len))
                hold on
                plot(obj.tArr(1:len), obj.poseErrArr(3, 1:len))
                hold on
                legend({'xErr', 'yErr', 'thetaErr'});
                
                figure
                plot(obj.tArr(1:len), obj.wArr(1:len), obj.traj.timeArray, ...
                    obj.traj.wArray)
                
                figure
                plot(obj.tArr(1:len), obj.vArr(1:len), obj.traj.timeArray, ...
                    obj.traj.VArray)
                
            end
            
            erF = obj.traj.getPoseAtTime(obj.tf) - robotPose(1:3);
            disp('terminal error x')
            disp(erF(1));
            disp('terminal error y')
            disp(erF(2));
            disp('terminal error theta')
            disp(erF(3));
        end
    end
end