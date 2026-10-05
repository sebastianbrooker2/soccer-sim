function [f, J] = dynamics(obj, t, x, u)
%DYNAMICS Ground-plane dynamics, constant velocity with optional rolling-friction decay.
%   dx/dt = vx, dy/dt = vy, dvx/dt = -KAPPA*vx, dvy/dt = -KAPPA*vy.
%   KAPPA = 0 (the default) is plain constant velocity, which is what
%   robots use. KAPPA > 0 models rolling friction as an exponential
%   velocity decay, a first-order simplification of a real ball's
%   sliding-then-rolling friction. Kicks and any other unmodelled
%   accelerations are still folded into the process noise (see
%   PROCESSNOISE) rather than simulated here.

f = [x(3); x(4); -obj.Kappa*x(3); -obj.Kappa*x(4)];

if nargout >= 2
    J = [0 0 1 0; ...
         0 0 0 1; ...
         0 0 -obj.Kappa 0; ...
         0 0 0 -obj.Kappa];
end
