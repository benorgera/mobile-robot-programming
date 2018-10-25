function [x,y,th] = irToXY(i,r)
    thOffset = atan2(0.38,4.23);
    th = deg2rad((i-1));
    th = th-thOffset;
    x =  r*cos(th);
    y =  r*sin(th);


end


