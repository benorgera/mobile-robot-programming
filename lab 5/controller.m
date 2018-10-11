classdef controller < handle
    % instantiated from trajectoryFollower
    properties
        traj
        tf
        tow
        feedback
        sleep
        tdelay
        tArr
        xArr
        yArr
        vArr
        wArr
        thetaArr
        xErrArr
        yErrArr
        thetaErrArr
        debug
        sim
        logLength = 1000;
        logIndex = 1;
    end
    methods
        function obj = controller(traj, sleep, tdelay, feedback, debug, sim, tow)
            %  follows a reference trajectory for feed forward control,
            %  with proportional feedback
            obj.tf = traj.tf;
            obj.traj = traj;
            obj.feedback = feedback;
            obj.sleep = sleep;
            obj.tdelay = tdelay;
            obj.debug = debug;
            obj.sim = sim;
            obj.tow = tow;
            obj.xErrArr = zeros(obj.logLength, 1);
            obj.yErrArr = zeros(obj.logLength, 1);
            obj.thetaErrArr = zeros(obj.logLength, 1);
            obj.yArr = zeros(obj.logLength, 1);
            obj.tArr = zeros(obj.logLength, 1);
            obj.vArr = zeros(obj.logLength, 1);
            obj.wArr = zeros(obj.logLength, 1);
            obj.thetaArr = zeros(obj.logLength, 1);
            obj.xArr = zeros(obj.logLength, 1);
        end
        
        function execute(obj)
            p = [];
            f = [];
            erW = [];
            erR = [];
            f = figure;
            p = plot(obj.xArr, obj.yArr, 'b-');
            axis([-1.5 1.5 -1.5 1.5])
            hold on
            plot(obj.traj.xArr, obj.traj.yArr, 'r-');
            hold on
            
            % communicate with encoder callbacks
            global robot;
            global robotPose;
            
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
            robot.encoders.NewMessageFcn=@encoderEventListener;
            
            % initialize feedback P controller coefficients
            kx = 1/obj.tow;
            ktheta = kx;
            
            tic
            while (true)
                t = toc;
                if t >= obj.tf + 2
                    break
                end
                
                rx = robotPose(1);
                ry = robotPose(2);
                rtheta = robotPose(3);
                
                V = 0.0;
                w = 0.0;
                refVels = obj.traj.getVelsAtTime(max(0, min(t+obj.tdelay, obj.tf)));
                refV = refVels(1);
                % last 1 second will just be feedback
                if t < obj.tf
                    V = refV;
                    w = refVels(2) * 10; % this is so damn weird
                end
                
                % need actual pose to compute error
                refPose = obj.traj.getPoseAtTime(max(0, min(t+obj.tdelay, obj.tf)));
                
                % this constant is dependent on vel
                ky = 0.0;
                if (refV ~= 0)
                    ky = 2 / (abs(refV)*obj.tow*obj.tow);
                end
                
                % error vector in world frame
                er = refPose - [rx; ry; rtheta];
                erTheta = atan2(sin(er(3)), cos(er(3)));
                s = sin(rtheta);
                c = cos(rtheta);
                rthetaNorm = atan2(s, c);
                
                erVectorRobot = [ 0 ; 0];
                if (rtheta ~= 0 && rthetaNorm ~= 0)
                    erVectorRobot = [c -s; s c] \ [er(1) ; er(2)];
                    % rotate to robot coords (A\b is same as A^-1 b)
                end
                
                % compute feedback, accounting for angular error
                uv = kx * erVectorRobot(1);
                uw = ky * erVectorRobot(2) + erTheta * ktheta;
                
                if (obj.feedback)
                    
                    disp('raw feedback')
                    disp([uv uw])
                    V = V + uv;
                    w = w + min(uw, 1.5);
                end
                
                if (obj.logIndex <= obj.logLength)
                    if (obj.debug)
                        figure(f)
                        delete(erW);
                        delete(erR);
                        erW = quiver(rx, ry, er(1), er(2), 0);
                        erR = quiver(0, 0, erVectorRobot(1), erVectorRobot(2));
                    end
                    
                    obj.xArr(obj.logIndex) = rx;
                    obj.yArr(obj.logIndex) = ry;
                    set(p,'Xdata',obj.xArr)
                    set(p,'Ydata',obj.yArr)
                    obj.thetaArr(obj.logIndex) = rtheta;
                    obj.tArr(obj.logIndex) = t;
                    obj.vArr(obj.logIndex) = robotPose(4);
                    obj.wArr(obj.logIndex) = robotPose(5);
                    obj.logIndex = obj.logIndex + 1;
                    obj.xErrArr(obj.logIndex) = erVectorRobot(1);
                    obj.yErrArr(obj.logIndex) = erVectorRobot(2);
                    obj.thetaErrArr(obj.logIndex) = erTheta;
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
            
            figure
            plot(obj.tArr, obj.xArr)
            hold on
            plot(obj.tArr,obj.yArr)
            hold on
            plot(obj.tArr, obj.thetaArr)
            hold on
            plot(obj.traj.tArr, obj.traj.xArr)
            hold on
            plot(obj.traj.tArr, obj.traj.yArr)
            hold on
            plot(obj.traj.tArr, obj.traj.thetaArr)
            hold on
            legend({'xRob', 'yRob', 'thetaRob', 'xRef', 'yRef', 'thetaRef'});
            
            figure
            plot(obj.tArr, obj.xErrArr)
            hold on
            plot(obj.tArr, obj.yErrArr)
            hold on
            plot(obj.tArr, obj.thetaErrArr)
            hold on
            legend({'xErr', 'yErr', 'thetaErr'});
            
            figure
            plot(obj.tArr, obj.wArr, obj.traj.tArr, obj.traj.wArr)
            
            figure
            plot(obj.tArr, obj.vArr, obj.traj.tArr, obj.traj.vArr)
            
            
        end
    end
end