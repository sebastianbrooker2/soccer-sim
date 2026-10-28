function [d2S, d2R, ldS] = gateStats(m, sys, y)
%GATESTATS Squared Mahalanobis distance of measurement Y to track SYS's
%   predicted measurement, linearised at the track mean. D2S uses the
%   innovation covariance S = H*P*H' + R (track uncertainty AND sensor
%   noise); D2R uses R alone.

mu = sys.density.mean();
P  = sys.density.cov();
[h, H] = m.predict(mu, sys);
R  = m.noiseDensity(sys).cov();
S  = H*P*H.' + R;
S  = (S + S.') / 2;
nu = y - h;
d2S = nu.' * (S \ nu);
ldS = log(max(det(S), realmin)); % log-determinant of the innovation covariance (for likelihood-style costs)
d2R = nu.' * (R \ nu);
end
