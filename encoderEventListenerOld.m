function encoderEventListenerOld(~,event)

global robot;
global encT;
global encTp;
global encX;
global encXp;
global encY;
global encYp;
global rx;
global ry;
global rtheta;

encTp = encT;
encT = double(event.Header.Stamp.Sec) + ...
    double(event.Header.Stamp.Nsec)/1000000000.0;
encXp = encX;
encX = robot.encoders.LatestMessage.Vector.X;
encYp = encY;
encY = robot.encoders.LatestMessage.Vector.Y;
dt = encT - encTp;
vl = (encX - encXp) / dt;
vr = (encY - encYp) / dt;
w = (vr - vl) / W;
v = (vl + vr) / 2;
delTheta = w * dt;
temp = rtheta + delTheta / 2;
dx = v * cos(temp) * dt;
dy = v * sin(temp) * dt;
rx = rx + dx;
ry = ry + dy;
rtheta = rtheta + delTheta;
end