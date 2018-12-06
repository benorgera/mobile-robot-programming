function [x,y,th] = irToXY(i,r)
    thOffset = -2 * pi / 180;
    th = deg2rad((i-1));
    th = th+thOffset;
    x =  r*cos(th);
    y =  r*sin(th);
end


