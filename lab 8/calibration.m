robot = raspbot("bot9");
robot.startLaser();
pause(5)

% thOffset = atan2(0.38,4.23);
arr = robot.laser.LatestMessage.Ranges;
arr = circshift(arr, -6);

x = [];
y = [];
th = [];
figure
axis([-4 4 -4 4]);
while true
    arr = robot.laser.LatestMessage.Ranges;
    arr = circshift(arr, 5);
    x = [];
    y = [];
    th = [];

    for i = 1:360
        tmpRange = arr(i);
        if tmpRange >=0.06 && tmpRange <=1
            [tmpx,tmpy,tmpth] = irToXY(i,tmpRange);

        else 
            tmpx = 0;
            tmpy = 0;
            tmpth = 0;
        end

        th(end+1) = tmpth;
        x(end+1) = tmpx;
        y(end+1) = tmpy;
    end

    
    scatter(-y,x,'rx');
    pause(0.02)
end

robot.shutdown();