function uref = trapezoidalVelocityProfile( t , amax, vmax, dist, sgn)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of
% duration tf. Sgn is the sign of the desired velocities.
% Returns 0 if t is negative.

tramp = vmax/amax;
tf = (dist + (vmax^2)/amax)/vmax;

if t < 0
    uref = 0;
elseif t < tramp
    uref = amax * t;
elseif (tf - t) < tramp
        uref = amax*(tf-t);
elseif (t> tramp && t < (tf-tramp))
    uref = vmax;
else
    uref = 0;
end
if uref<0
    uref = 0
end
uref = sgn*uref;

end