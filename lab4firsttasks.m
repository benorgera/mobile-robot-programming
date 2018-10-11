%% simulated 0.1 m drive
robot = raspbot('sim');
pause(5)
robot.stop()
pause(2)
global encX0 % used to get xl
global encY0 % used to get xrs
global encTP % used to get dt
encX0 = robot.encoders.LatestMessage.Vector.X;
encY0 = robot.encoders.LatestMessage.Vector.Y;
encTP = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + ...
  double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1e9;
encXP = encY0;
encYP = encY0;

f = figure;
xarr = [];
yarr = [];
velPlot = plot(xarr, yarr, '-b');

tPrevPath = 0;
pathIntegral = 0;
global tf;
global tr;
dist = 1;
vmax = 0.25;
amax = 0.75;
tf = (dist + vmax * vmax / amax) / vmax;
tr = vmax/amax;
d = 0;
goal = 0.3;
P = 4.5;
I = 0.0;
D = 0.3;
vm = 0.25;
am = 0.75;
tr = vm / am;
pause(0.005)
[e, dt] = eofT(robot, goal);
ePrev = e; % prev error used for derivative

tic

while toc < 6.0
% integral = integral + e * dt;
% if dt ~= 0.0
%     d = (e - ePrev) / dt;
% end
% u = P * e + I * integral + D * d;
% ePrev = e;
% if u > 0.3
%     u = 0.3;
% end
t = toc;
vel = trapezoidalVelocityProfile(t, amax, vmax, dist, true);
pathIntegral = pathIntegral + (toc - tPrevPath) * vel;
disp(vel)
disp(get(velPlot, 'xdata'))
set(velPlot, 'xdata', [get(velPlot, 'xdata') t], ...
    'ydata', [get(velPlot, 'ydata') vel]);
robot.sendVelocity(vel, vel)
pause(0.005)
% [e, dt] = eofT(robot, goal);
end
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

function uref = trapezoidalVelocityProfile(t, amax, vmax, dist, pos)
global tf
global tr
global velPlot
if t < tr
    uref = amax * t;
elseif t < tf - tr
    uref = vmax;
elseif tf - t < vmax / amax
    uref = (tf - t) * amax;
else
    uref = 0;
end
if ~pos
    uref = -uref;
end
end