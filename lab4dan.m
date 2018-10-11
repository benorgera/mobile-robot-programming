close all; clear; clc;
% Setup world.
% lineObject instances represent bodies.
walls = lineObject();
% Specify points as an n x 2 array.
% Simulator length units: m.
walls.lines = [1.5 0; 3 1.5; 1.5 3; 0 1.5; 1.5 0];
obstacle = lineObject();
obstacle.lines = [1.6 1.4; 1.6 1.6; 1.4 1.6; 1.4 1.4; 1.6 1.4];

% Environments are specified as lineMap instances.
% lineMap constructor takes an array of lineObject instances as input.
map = lineMap([walls obstacle]);
% Show the map.
hf = map.plot();

% Fire simulator.
% Close map plot.
if ishandle(hf)
    close(hf);
end

% By default robot starts at pose x = 0, y = 0, theta = 0.
%rob = raspbot('sim');
% Start at desired pose.
%robot = raspbot('sim',[1; 1.5; 0]);
%robot.genMap(map.objects);
%%
close all; clear; clc;

robot = raspbot();

initialized = false;
firstTime = true;
goalDistance = 1.;
stopTime = 6;
errorTolerance = 0.0001;

p_gain = 2.5;
d_gain = 0.4;
i_gain = 0.01;
t_delayed = 0.37;%50;
MODE = 1;

lastError = 0;

errorArray(1) = 0;
distanceArray(1) = 0;
timeArray(1) = 0;
actDistArray(1) = 0;%encoder distance
urefArray(1) = 0;
figure;

distancePlot = plot(timeArray, distanceArray , 'b-', 'DisplayName', 'Integrated distance');
hold on;
actDistPlot  = plot(timeArray, actDistArray(1) , 'g-',  'DisplayName', 'Actual distance');
hold on;
urefPlot    = plot(timeArray, urefArray(1) , 'r-',  'DisplayName', 'uref');
xlim([0, 6]);
ylim([0, 1100]);
legend('show')

title('distance vs time')
xlabel('time in secs');
ylabel('Distance in meters');

figure;
errorPlot    = plot(timeArray, errorArray(1) , 'r-',  'DisplayName', 'Error');

title('Error over time')
xlabel('Time in secs');
ylabel('Error in meters');
%xlim([0, 6]);
%ylim([-0.2, 1.1]);
legend('show')


pause(0.5)
%pause(2.);



s = 0;
dist = 1.; 
amax = 3*0.25; 
vmax = 0.25;
sgn=1;
tf = (dist + (vmax^2)/amax)/vmax;

t_prev = 0;
t = 0;

uref_delayed = 0;
s_delayed = 0;


tick_count = 0;
e = 0;

sl_bias   = robot.encoders.LatestMessage.Vector.X;
sr_bias   = robot.encoders.LatestMessage.Vector.Y;
d_prev = 0;
myClock = tic;
t0 = toc(myClock);

while t < tf + 1 + t_delayed
    t  = toc(myClock)-t0;
    dt = t - t_prev;
    t_prev = t;

    if t > stopTime
        robot.stop();
        fprintf('stopping the robot\n');
        break
    end
    
    sl   = robot.encoders.LatestMessage.Vector.X - sl_bias;
    sr   = robot.encoders.LatestMessage.Vector.Y - sr_bias;    
    distance = (sl + sr)/2;
    v = (distance - d_prev)/dt;
    d_prev = distance;
    uref = trapezoidalVelocityProfile(t, amax, vmax, dist, sgn); 
    uref_delayed = trapezoidalVelocityProfile(t-t_delayed , amax, vmax, dist, sgn);
    
    if t > tf + t_delayed
        uref_delayed = 0;
        uref = 0;
    end
    s_delayed = s_delayed + uref_delayed*dt;
    fprintf('actual distance : %.5f delayed distance: %.2f \n', 1000*distance, 1000*s_delayed);
    
       
    error = s_delayed - distance;
    e = e + error;
    errorDerivative = (error - lastError)/dt;
    lastError = error;
    
    if firstTime
        errorIntegral = 0;
        firstTime = false;
    end
    errorIntegral   = errorIntegral + error * dt;
    
    control = uref + MODE*(error * p_gain + errorDerivative * d_gain + errorIntegral * i_gain);

    if control > .3
        control = .3;
    end
    robot.sendVelocity(control, control);

    set(distancePlot, 'xdata', [get(distancePlot,'xdata') t],...
    'ydata', [get(distancePlot,'ydata') s_delayed*1000]);
    set(actDistPlot, 'xdata', [get(actDistPlot,'xdata') t],...
    'ydata', [get(actDistPlot,'ydata') distance*1000]);
    set(errorPlot, 'xdata', [get(errorPlot,'xdata') t],...
    'ydata', [get(errorPlot,'ydata') (s_delayed-distance)*1000]);
    set(urefPlot, 'xdata', [get(urefPlot,'xdata') t],...
    'ydata', [get(urefPlot,'ydata') uref*1000]);
    
    pause(0.005)
    tick_count = tick_count + 1;
    
end
fprintf('avg error: %.4f\n', (e/tick_count)*1000);
robot.stop()
robot.shutdown();

%%
close all; clear; clc;
my_clock = tic;
dist = 1.; 
amax = 3*0.25; 
vmax = 0.25;
sgn=1;
tf = (dist + (vmax^2)/amax)/vmax;
vArray(1) = 0;
tArray(1) = 0;
sArray(1) = 0;
figure;
vPlot = plot(tArray, vArray , 'b-');
xlim([0 4.5]);
ylim([0 1]);
title('speed vs time')
xlabel('time in secs');
ylabel('speed in m/sec');

t0 = toc(my_clock);
t = 0;
s = 0;
t_prev = 0;

while t<tf
    t  = toc(my_clock)-t0;
    dt = t-t_prev;
    t_prev = t;
    uref = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
    s = s + uref*dt
    
    set(vPlot, 'xdata', [get(vPlot,'xdata') t],...
    'ydata', [get(vPlot,'ydata') s]);
    pause(0.005);
end


