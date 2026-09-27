function [f, J] = dynamics(obj, t, x, u)
%DYNAMICS Constant-velocity ground-plane dynamics for the ball.
%   dx/dt = vx, dy/dt = vy, dvx/dt = 0, dvy/dt = 0 -- accelerations
%   (rolling friction, kicks, ...) are treated as unmodelled and folded
%   into the process noise (see PROCESSNOISE) rather than simulated here.

f = [x(3); x(4); 0; 0];

if nargout >= 2
    J = [0 0 1 0; ...
         0 0 0 1; ...
         0 0 0 0; ...
         0 0 0 0];
end
