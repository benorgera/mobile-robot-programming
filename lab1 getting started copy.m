%% Lab 1 Task 1 Move the Robot

clear all
pause(6)
robot = raspbot('sim');
robot.stop()

% robot.encoders.LatestMessage.Vector.X
leftArray = [];
leftArray1 = [];
timeArray = [];
rghtArray = [];
rghtArray1 = [];
leftEncoder = 3242;
l0 = leftEncoder;
l01 = robot.encoders.LatestMessage.Vector.X - 20;
rghtEncoder = 5433;
r0 = rghtEncoder - 10;
r01 = robot.encoders.LatestMessage.Vector.Y - 30;
i = 1;
v = 0.05; % m/s
sleep = 50 / 1000; % s

tic
while (leftEncoder - l0 + rghtEncoder - r0) / 2 < 304.8
    robot.sendVelocity(v, v) % 5 cm/s
    pause(sleep) % 50 ms
    delta = v * 1000 * sleep; % mm
    rghtEncoder = round(rghtEncoder + delta);
    leftEncoder = round(leftEncoder + delta);
    leftArray(i) = leftEncoder - l0;
    leftArray1(i) = robot.encoders.LatestMessage.Vector.X - l01;
    rghtArray(i) = rghtEncoder - r0;
    rghtArray1(i) = robot.encoders.LatestMessage.Vector.Y - r01;
    timeArray(i) = toc;
    i = i + 1;
    figure(2)
    plot(timeArray, leftArray, timeArray, rghtArray, timeArray, leftArray1, timeArray, rghtArray1)
    figure(1)
end

robot.stop()

% while ((leftEncoder - l0) / 1000 + (rghtEncoder - r0) / 1000) / 2 > 0
%     robot.sendVelocity(-0.05, -0.05) % 5 cm/s
%     pause(0.05) % 50 ms
%     rghtEncoder = rghtEncoder - 0.05 * 50;
%     leftEncoder = leftEncoder - 0.05 * 50;
% end

robot.stop()
robot.shutdown()