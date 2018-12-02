gain = 0.3;
        errThresh = 0.01;
        gradThresh = 0.0005;

% Set up lines
p1 = [-2 ; -2];
p2 = [ 2 ; -2];
p3 = [ 2 ; 2];
p4 = [-2 ; 2];
lines_p1 = [p1 p2 p3 p4];
lines_p2 = [p2 p3 p4 p1];
% Set up test points
nPts = 10;
x1 = -2.0*ones(1,nPts);
x2 = linspace(-2.0,2.0,nPts);
x3 = 2.0*ones(1,nPts);
y1 = linspace(0.0,2.0,nPts);
y2 = 2.0*ones(1,nPts);
y3 = linspace(2.0,0,nPts);
w = ones(1,3*nPts);
x1pts = [x1 x2 x3];
y1pts = [y1 y2 y3];
w1pts = w;
modelPts = [x1pts ; y1pts ; w1pts];
% pick a pose
dx = -0.1*rand();
dy = -0.1*rand();
dt = -0.1+0.2*rand();
p0 = pose(0.0+dx,0.0+dy,0.0+dt);
curPose = p0;

local = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh);

[err, grad] = local.getJacobian(p0,modelPts);
gradMag = norm(grad);
i = 1;

disp("err0")
disp(err)
disp("gradMag0")
disp(gradMag)

figure
while err > errThresh || gradMag > gradThresh
    curPose = pose(curPose.getPoseVec() + -gain * grad);
    hold off
    plot(lines_p1, lines_p2)
    hold on
    
    worldPts = curPose.bToA()*modelPts;
    scatter(worldPts(1, :), worldPts(2, :), 'bx')
    
    [err, grad] = local.getJacobian(curPose,modelPts);
    disp(grad);
    gradMag = norm(grad);
    disp(i)
    disp(err)
    disp(gradMag)
    i = i + 1;
end


disp("broke out")