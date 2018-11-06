classdef MRPLsystem < handle

    methods(Static)
        
        function acqPoseVec = acquisitionPose(flagPoseSensorCoords, extraOffset)
            
            flagToPalletFront = pose(-robotModel.palletFrontToFlag ...
                - extraOffset, 0, 0).bToA();
            
            % everything below is in robot coords
            flagPose = pose(flagPoseSensorCoords - [robotModel.frontOffset; 0; 0]);
            flagPoseVec = flagPose.getPoseVec();
            palletPose = flagToPalletFront * ...
                [flagPoseVec(1:2, 1); 1];
            acqPoseVec = [palletPose(1:2, 1); flagPoseVec(3)];
        end
        
    end
    
end

