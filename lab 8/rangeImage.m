classdef rangeImage < handle
    properties(Constant)
        
    end
    
    properties(Access = public)
        
    end
    
    methods(Static)
        function findLineCandidate()
            mat = zeros(10, 360);
            global robot
            
            % store 10 images
            for j = 1 : num
                arr = circshift(robot.laser.LatestMessage.Ranges, -6);
                for i = 1 : 360
                    mat(j, i) = arr(i);
                end
                pause(0.1)
            end
            
            goodOnes = mat > 0.06 & mat < 4.0;
            % this probably doesn't work with 10 rows
            % try a small attempt in the console and fix this syntax
            % accordingly
            mat = mat(goodOnes);
            indices = linspace(2,360,360);
            indices = indices(goodOnes);
            
            % loop through all range points, finding points within 12.7/2 cm
            % compute clouds coords of this cluster
            
            % not sure if this is intended for each point in the cloud
            % (what's x' vs x, maybe actual point vs centroid of cloud?)
            Ixx = x' * x;
            Iyy = y' * y;
            Ixy = - x' * y;
            Inertia = [Ixx Ixy;Ixy Iyy] / numPts; % normalized
            lambda = eig(Inertia);
            lambda = sqrt(lambda)*1000.0;
        end
    end
end