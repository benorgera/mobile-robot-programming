%% drive forward and record velocity (differentiate)
pause(1)
% all distance units in meters
% max robot vel is 0.5 m/s
global robot;
global encT;
global encTp;
global encX;
global encXp;
global encY;
global encYp;
global W;
global x;
global y;
global theta;
W = 0.084;
robot = raspbot();
robot.stop()

f = figure;
axis([0 5 -0.5 0.5])
hold on

pause(3)
encXp = robot.encoders.LatestMessage.Vector.X;
encYp = robot.encoders.LatestMessage.Vector.Y;
encTp = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + ...
    double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000.0;
t0 = encTp;
robot.encoders.NewMessageFcn=@encoderEventListener;

robot.sendVelocity(0.4, 0.4)
while encT - t0 < 5
    deltaT = encT - encTp;
    vl = (encX - encXp) / deltaT;
    vr = (encY - encYp) / deltaT;
    t = encT - t0;
    v = (vl + vr) / 2;
    plot(t, v, 'x')
    robot.sendVelocity(0.4, 0.4)
    pause(0.005)
end
robot.stop()
%% estimate curve position
f = figure(1);
axis([0 0.4 0 0.4])
hold on
pause(1)

W = 0.084;
tf = sqrt(32 * pi);
dt =  0.01; % s
v = 0.1;
kw = 0.125;

% w = kw * t
thet = 0.0;
t = 0.0;
xt = 0.0;
yt = 0.0;

while t + dt <= tf
    w = kw * t;
    dTheta = w * dt;
    term = W * w / 2;
    vr = v + term;
    vl = v - term;
    V = (vl + vr) / 2.0;
    thetaTemp = thet + dTheta / 2.0;
    dx = V * cos(thetaTemp) * dt;
    dy = V * sin(thetaTemp) * dt;
    thet = thet + dTheta;
    xt = xt + dx;
    yt = yt + dy;
    t = t + dt;
    plot(xt, yt, '.')
    hold on
    axis([0 0.4 0 0.4])
    pause(dt)
end

%% estimate figure 8 position
xArr = [];
yArr = [];
figure;
axis([0 0.6 0 0.6])
% myPlot = plot(xArr, yArr, 'b-');
% xlim([0.0 0.5]);
% ylim([0.0 0.5]);

W = 0.084;
v = 0.2;
sf = 1;
tf = sf / v;
kth = 2 * pi / sf;
kk = 15.1084;
ks = 3;
Tf = ks * tf;

% w = kw * t
theta = 0.0;
t = 0.0;
x = 0.0;
y = 0.0;
tic;
tp = 0.0;

while toc <= Tf
    T = toc;
    t = T / ks;
    dt = t - tp;
    s = v * t;
    K = kk / ks * sin(kth * s);
    w = K * v;
    
    dTheta = w * dt;
    thetaTemp = theta + dTheta / 2.0;
    dx = v * cos(thetaTemp) * dt;
    dy = v * sin(thetaTemp) * dt;
    theta = theta + dTheta;
    x = x + dx;
    y = y + dy;
    tp = t;
%      plot(x, y, '.')
%      hold on
%      axis([0 0.6 0 0.6])
     xArr = [xArr x];
     yArr = [yArr y];
%     set(myPlot, 'xdata', [get(myPlot,'xdata') x], ...
%         'ydata', [get(myPlot,'ydata') y]);
    pause(0.01)
end

plot(xArr, yArr, '.');

%% drive by figure 8
figure;
ax = [-0.5 0.5 -0.5 0.5];
axis(ax)

robot = raspbot();
robot.stop()

W = 0.084;
v = 0.2;
sf = 1;
tf = sf / v;
kth = 2 * pi / sf;
kk = 15.1084;
ks = 3;
Tf = ks * tf;

pause(2)
encXp = robot.encoders.LatestMessage.Vector.X;
encYp = robot.encoders.LatestMessage.Vector.Y;
encTp = double(robot.encoders.LatestMessage.Header.Stamp.Sec) + ...
    double(robot.encoders.LatestMessage.Header.Stamp.Nsec)/1000000000.0;
t0 = encTp;
robot.encoders.NewMessageFcn=@encoderEventListener;

% w = kw * t
theta = 0.0;
x = 0.0;
y = 0.0;
tic;
tp = 0.0;

while toc <= Tf
    T = toc;
    t = T / ks;
    dt = t - tp;
    s = v * t;
    K = kk / ks * sin(kth * s);
    w = K * v;
    term = W / 2 * w;
    vl = v - term;
    vr = v + term;
    robot.sendVelocity(vl, vr);
    tp = t;
    plot(x, y, '.');
    hold on
    axis(ax);
    pause(0.005)
end
robot.stop();
robot.encoders.NewMessageFcn=[];


% assume dt evenly divides the interval
function res = modelDiffSteerRobot(vl, vr, t0, tf, dt)
global W
res = [];
t = t0;
theta = 0.0;
x = 0.0;
y = 0.0;
V = (vl + vr) / 2.0;
w = (vr - vl) / W;
dTheta = w * dt;
while t < tf
    thetaTemp = theta + dTheta / 2.0;
    dx = V * cos(thetaTemp) * dt;
    dy = V * sin(thetaTemp) * dt;
    theta = theta + dTheta;
    x = x + dx;
    y = y + dy;
    t = t + dt;
    res = [res, [x y t]];
end 
end
    
