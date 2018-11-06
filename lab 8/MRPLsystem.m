classdef MRPLsystem < handle
    %MRPLSYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        flagToPalletFront = pose(-robotModel.palletFrontToFlag, 0, 0).bToA();
    end
    
    methods(Static)
        
        function acqPoseVec = acquisitionPose(flagPoseSensorCoords)
            
            % everything below is in robot coords
            flagPose = pose(flagPoseSensorCoords - [robotModel.frontOffset; 0; 0]);
            palletPose = MRPLsystem.flagToPalletFront * [flagPose(1:2, 1); 1];
            acqPoseVec = [palletPose(1:2, 1); flagPose(3)];
        end
        
    end
    
end

