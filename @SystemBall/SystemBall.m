classdef SystemBall < SystemEstimatorSimulator
%SYSTEMBALL Constant-velocity ground-plane process model for tracking a
%   dynamic ball from multiple robot observers.
%   State x = [x; y; vx; vy] -- field-frame position and velocity, m and
%   m/s. Robots/observers are NOT part of this system's state or
%   dynamics: only their own (uncertain) position is known, and that is
%   carried per-measurement by MEASUREMENTBALLPOSITION, not simulated
%   here.

    methods
        function obj = SystemBall()
            % Call superclass constructor(s)
            obj@SystemEstimatorSimulator();

            % Initial time
            obj.time = 0;

            % Initial simulator (ground-truth) state
            obj.x_sim = [ ...
                0; ...      % Initial x position
                0; ...      % Initial y position
                1; ...      % Initial x velocity
                0.5 ...     % Initial y velocity
                ];

            % Initial estimator state (prior)
            mu0 = [0; 0; 0; 0];
            S0 = diag([5, 5, 2, 2]) / 3;
            obj.density = GaussianInfo.fromSqrtMoment(mu0, S0);
        end

        [f, J] = dynamics(obj, t, x, u)
        u = input(obj, t, x)
        [pdw, idx] = processNoise(obj, dt)
    end
end
