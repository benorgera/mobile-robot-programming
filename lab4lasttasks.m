%% simulated 0.1 m drive
robot = raspbot();
robot.stop()
pause(2)
global encX0 % used to get xl
global encY0 % used to get xr
global encTP % used to get dt
encX0 = robot.encoders.LatestMessage.Vector.X;
encY0 = robot.encoders.LatestMessage.Vector.Y;
encTP = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + ...
  double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
encXP = encY0;
encYP = encY0;
tic
integral = 0;
d = 0;
goal = 0.3;
P = 4.5;
I = 0.0;
D = 0.3;
pause(0.005)
[e, dt] = eofT(robot, goal);
ePrev = e; % prev error used for derivative
figure(1)

disp(e)


while toc < 6.0
integral = integral + e * P;
if dt ~= 0.0
    d = (e - ePrev) / dt;
end
u = P * e + I * integral + D * d;
ePrev = e;
if u > 0.3
    u = 0.3;
end
disp(u)
robot.sendVelocity(u, u)
pause(0.005)
[e, dt] = eofT(robot, goal);
end
disp(e)
robot.stop()

function [e, dt] = eofT(robot, desired)
global encX0
global encY0
global encTP
e = desired - (robot.encoders.LatestMessage.Vector.X - encX0 + ...
    robot.encoders.LatestMessage.Vector.Y - encY0) / 2;
t = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + ...
  double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
figure(1);
plot(t, e, '.');
hold on;
dt = t - encTP;
encTP = t;
end