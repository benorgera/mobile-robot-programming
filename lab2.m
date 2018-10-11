%% Lab 2
pause(1)
robot = raspbot();
robot.stop()
robot.startLaser()
pause(5)
tic

f = figure;
axis([-2 2 -2 2])
hold on
prev = plot(0, 0, 'x');
vl = 0;
vr = 0;

idealRange = 0.5; % m
gain = 0.7;
dropTol = 0.6; % s
W = 0.09; % m

while true
    L = 1;
    ind = -1;
    for i = [276:360 1:96]
        r = double(robot.laser.LatestMessage.Ranges(i));
        if r >= 0.06 && r <= 1 && r < L
            L = r;
            ind = i;
        end
        
    end
    if ind ~= -1
        tic
        [x, y, th] = irToXy(ind, L);
        V = gain * (L - 0.5);
        K = x / (L * y);
        w = K * V;
        vl = V - w * W / 2;
        vr = V + w * W / 2;
        robot.sendVelocity(vl, vr)
        delete(prev)
        prev = plot(-y, x, 'x');
    elseif toc > dropTol
        robot.stop()
    else
        robot.sendVelocity(vl, vr)
    end
    pause(0.2)
end
%% Lab 2 Plot Lidar
pause(4)
robot = raspbot();
robot.stop()
robot.startLaser()
pause(3)

f = figure;
axis([-2 2 -2 2])
hold on
prev = plot(0, 0, 'x');

while true
    for i = [276:360 1:96]
        r = double(robot.laser.LatestMessage.Ranges(i));
        if r < 1
            [x, y, th] = irToXy(i, r);
            plot(-y, x, 'x')
            hold on
        end
    end
    hold off
    pause(0.2)
end

function [x, y, th] = irToXy(i, r)
th = (i - 6.0) * pi / 180;
x = r * cos(th);
y = r * sin(th);
end
