classdef trajectoryString < handle
    properties(Access = public)
        timeArray = []
        poseArray = []
        VArray = []
        wArray = []
        tf = 0
    end
    methods
        function obj = trajectoryString(trajectoryArr)
            % sum up some trajectories
            obj.tf = 0;
            curPose = pose(0, 0, 0);
            
            for i = 1:length(trajectoryArr)
                disp("here")
                curPoseVec = curPose.getPoseVec();
                disp(curPoseVec);
                curTraj = trajectoryArr(i);
                % assume v,w and pose arrays are same length
                obj.VArray = [obj.VArray curTraj.VArray];
                obj.wArray = [obj.wArray curTraj.wArray];
                
                transform = curPose.bToA();
                
                for j = 1:length(curTraj.poseArray)
                    xAndY = transform * [curTraj.poseArray(1:2,j); 1];
                    theta = curTraj.poseArray(3,j) + curPoseVec(3);
                    vec = [xAndY(1:2); theta];
                    obj.poseArray = [obj.poseArray, vec];
                    if j == 1
                        obj.timeArray = [obj.timeArray, ...
                            curTraj.timeArray(j) + obj.tf + 0.000001];
                    else
                        obj.timeArray = [obj.timeArray, ...
                            curTraj.timeArray(j) + obj.tf];
                    end
                end
                
                curPose = pose(vec);
                obj.tf = obj.tf + curTraj.getTrajectoryDuration();
            end
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