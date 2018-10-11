function encoderEventListener(~,event)

% read only (from another declaration
global robot;
global robotPose;

% only used by this function
global encT;
global encTp;
global encX;
global encXp;
global encY;
global encYp;

% integrates velocities to get pose as new data is available, so
% you don't run the risk of reading the same data twice

encT = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/1000000000.0;
encX = robot.encoders.LatestMessage.Vector.X;
encY = robot.encoders.LatestMessage.Vector.Y;
dt = encT - encTp;
dEncX = encX - encXp;
dEncY = encY - encYp;
if isequal(encTp > 0, []) % hack to see if initialized (empty logical array)
    dt = 0.0;
    dEncX = 0.0;
    dEncY = 0.0;
end
encTp = encT;
encXp = encX;
encYp = encY;

% disp('in callback')
% disp(rx);
% disp(ry);
% disp(rtheta);

if (dt ~= 0.0)
    rx = robotPose(1);
    ry = robotPose(2);
    rtheta = robotPose(3);
    
    vl = dEncX / dt;
    vr = dEncY / dt;
    [v, w] = robotModel.vlvrToVw(vl, vr);
    delTheta = w * dt;
    temp = rtheta + delTheta / 2.0;
    dx = v * cos(temp) * dt;
    dy = v * sin(temp) * dt;
    robotPose(1) = rx + dx;
    robotPose(2) = ry + dy;
    robotPose(3) = rtheta + delTheta;
    robotPose(4) = v;
    robotPose(5) = w;
    
else
    disp('encoders starting')
end
end