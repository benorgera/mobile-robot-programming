classdef MRPLsystem < handle
    
    methods(Static)
        
        function acqPoseVec = acquisitionPose(sailToSensorVector, extraOffset, debug)
            
            % transfrom from c to d (c is subscript) encodes vector from d
            % to c
            %             robotToSailVector = sailToSensorVector - [robotModel.frontOffset + extraOffset; 0; 0];
            %             sailToRobotTransform = pose(robotToSailVector).bToA();
            %
            %             sailToPalletFrontVector = [-robotModel.palletFrontToSail; 0; 0];
            %             palletFrontToSailTransform = pose(sailToPalletFrontVector).bToA();
            %
            %             palletFrontToRobotTransform = sailToRobotTransform * palletFrontToSailTransform;
            %             robotToPalletFrontVector = pose.matToPoseVec(palletFrontToRobotTransform);
            %
            %             acqPoseVec = [robotToPalletFrontVector(1:2); sailToSensorVector(3)];
            
            robotToSailVector = sailToSensorVector - [robotModel.frontOffset; 0; 0];
            
            totalOffset = robotModel.palletFrontToSail + extraOffset;
            theta = robotToSailVector(3);
            theta = atan2(sin(theta), cos(theta));
            shift = [totalOffset * cos(theta); totalOffset * sin(theta); 0];
            
            robotToPalletFront1 = robotToSailVector + shift;
            robotToPalletFront2 = robotToSailVector - shift;

            if norm(robotToPalletFront1) < norm(robotToPalletFront2)
                acqPoseVec = robotToPalletFront1;
            else
                acqPoseVec = robotToPalletFront2;
            end
            
            if acqPoseVec(1) < 0
                acqPoseVec(3) = atan2(sin(acqPoseVec(3) + pi), ...
                    cos(acqPoseVec(3) + pi));
            end
            
            if debug
                scatter([acqPoseVec(1)], [acqPoseVec(2)], 'rx')
                palletFrontXWorld = robotModel.frontOffset + acqPoseVec(1);
                palletFrontYWorld = acqPoseVec(2);
                plot([robotModel.frontOffset palletFrontXWorld] ...
                    , [0 palletFrontYWorld])
                thetaParallel = acqPoseVec(3) + pi/2;
                halfY = sin(thetaParallel) * 0.05;
                halfX = cos(thetaParallel) * 0.05;
                plot([palletFrontXWorld - halfX palletFrontXWorld + halfX], ...
                    [palletFrontYWorld - halfY palletFrontYWorld + halfY])
                
                flagX = sailToSensorVector(1);
                flagY = sailToSensorVector(2);
                plot([flagX - halfX flagX + halfX], ...
                    [flagY - halfY flagY + halfY])
            end
        end
        
    end
    
end

