function [pdw, idx] = processNoise(obj, dt)
%PROCESSNOISE Process noise increment dw ~ N(0, Q*dt) injected on [vx; vy].
%   Represents the unmodelled accelerations (rolling friction, kicks)
%   that the constant-velocity DYNAMICS doesn't capture.

% Square-root power spectral density of continuous-time process noise.
% Std of the unmodelled ball acceleration (rolling friction, kicks) the
% constant-velocity DYNAMICS doesn't capture, in m/s^2 per sqrt(Hz). Was
% tuned between 2.0 and 4.0 while chasing two opposite failure modes: too
% low left the filter overconfident (and so resistant to correction)
% through the run scripts' scripted kicks; too high, combined with tight
% measurement noise, made the prior so wide that BFGSTrustSqrt couldn't
% converge on the likelihood peak within its iteration budget. 2.5 is a
% middle ground between them.
SQ = diag([0.1, 0.1]);

% Indices of process model equations where process noise is injected
idx = [3, 4];

% Distribution of noise increment dw ~ N(0, Q*dt) for time increment dt.
% dt == 0 (e.g. two sensors sharing a timestamp) is now guarded upstream
% in SystemEstimator.predict, which skips calling this at all when
% dt == 0, so this never has to handle a singular SQ*sqrt(0) itself.
pdw = GaussianInfo.fromSqrtMoment(SQ*realsqrt(dt));
