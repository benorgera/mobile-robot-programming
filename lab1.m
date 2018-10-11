%% Lab 1 Task 1 Move the Robot

clear all
pause(6)
robot = raspbot();
robot.stop()
pause(1)

leftArray = [];
timeArray = [];
rghtArray = [];
l0 = robot.encoders.LatestMessage.Vector.X;
r0 = robot.encoders.LatestMessage.Vector.Y;
i = 1;
v = 0.05; % m/s
sleep = 50 / 1000; % s

figure
tic
while avgDist(robot, l0, r0) < 0.3048
    robot.sendVelocity(v, v) % 5 cm/s
    pause(sleep) % 50 ms
    leftArray(i) = robot.encoders.LatestMessage.Vector.X - l0;
    rghtArray(i) = robot.encoders.LatestMessage.Vector.Y - r0;
    timeArray(i) = toc;
    i = i + 1;
    % figure(2)
    plot(timeArray, leftArray, timeArray, rghtArray)
    % figure(1)
end

while avgDist(robot, l0, r0) > 0
    robot.sendVelocity(-v, -v) % 5 cm/s
    pause(sleep) % 50 ms
    leftArray(i) = robot.encoders.LatestMessage.Vector.X - l0;
    rghtArray(i) = robot.encoders.LatestMessage.Vector.Y - r0;
    timeArray(i) = toc;
    i = i + 1;
    % figure(2)
    plot(timeArray, leftArray, timeArray, rghtArray)
    % figure(1)
end

robot.stop()
robot.shutdown()

function d = avgDist(robot, l0, r0)
le = robot.encoders.LatestMessage.Vector.X;
re = robot.encoders.LatestMessage.Vector.Y;
d = (le + re - l0 - r0) / 2;
end