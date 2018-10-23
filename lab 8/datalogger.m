%% snapshot 10 arrays every 10 second beep
robot = raspbot();
robot.startLaser()
pause(5)

num = 10; % num of images

while true
    figure
    axis([-4 4 -4 4]);
    
    pause(10)
    mat = zeros(10, 360);
    beep
    
    for j = 1 : num
        arr = circshift(robot.laser.LatestMessage.Ranges, -6);
        for i = 1 : 360
            mat(j, i) = arr(i);
        end
        pause(0.1)
    end
    beep
    pause(0.1)
    beep
    
    xyMat = zeros(2, 360 * num);
    z = 1;
    for j = 1 : num
        for i = 1 : 360
            r = mat(j, i);
            xyMat(:, z) = [ r * cos(i) ; r * sin(i) ];
            z = z + 1;
        end
    end
    plot(xyMat(1, :), xyMat(2, :)) 
end

