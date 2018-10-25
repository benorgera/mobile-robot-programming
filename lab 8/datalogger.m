%% snapshot 10 arrays every 10 second beep
robot = raspbot("raspbot9");
robot.startLaser();
pause(5)

num = 10; % num of images



    
pause(1)
mat = zeros(10, 360);
beep
disp("start");
for j = 1 : num
    arr = circshift(robot.laser.LatestMessage.Ranges, 5);
    for i = 1 : 360
        mat(j, i) = arr(i);
    end
    disp("finish one");
    disp(j);
    beep
    pause(10)
    beep
end
disp("finish all!!!!");
beep
pause(0.5)
beep
pause(0.5)
beep


xyMat = zeros(3, 360 ,num);
for j = 1 : num
    for i = 1 : 360
        r = mat(j, i);
        xyMat(:, i, j) =  irToXY(i,r);
    end
end
save('data.mat','xyMat');


