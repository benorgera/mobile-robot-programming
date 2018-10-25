classdef MRPLsystem < handle
    %MRPLSYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = MRPLsystem()

        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
    methods(Static)
        
        function acqPoseVec = acquisitionPose(objPoseVec,robFrontOffset,objFaceOffset,moreOffset)
            targetX = objPoseVec(1);
            targetY = objPoseVec(2);
            targetTh = objPoseVec(3);
            acqPoseVec = [0,0,0];
            Trs = [1,0,0; 0,1,0; 0,0,1];
            Tso = [cos(targetTh),-sin(targetTh),targetX;sin(targetTh),cos(targetTh),targetY;0,0,1];
            Tog = [1,0,-objFaceOffset-robFrontOffset+moreOffset; 0,1,0; 0,0,1];
            Trg = Trs*Tso*Tog;
            tmpPosVec = objPoseVec;
            tmpPosVec(3)=1;
            realTh = atan2(Trg(2,1),Trg(1,1));
%             acqPoseVec = Trg*tmpPosVec';
            acqPoseVec(1) = Trg(1,3);
            acqPoseVec(3) = Trg(2,3);
            acqPoseVec(3) = realTh;
        end
        
    end
    
end

