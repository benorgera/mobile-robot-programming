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

% why does initializing those globals break the plotting?

% why does using globals in the second section not work?


figure(1);
ax = [-0.5 0.5 -0.5 0.5];
axis(ax)

robot = raspbot();
robot.stop()

W = 0.09;
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
% encX = 0;
% encY = 0;
% encT = 0;
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
    figure(1);
    disp(x);
    disp(y);
    plot(x, y, '.');
    hold on
    axis(ax);
    pause(0.005)
end
robot.stop();
robot.encoders.NewMessageFcn=[];