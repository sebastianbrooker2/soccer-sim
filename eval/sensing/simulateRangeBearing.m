function y = simulateRangeBearing(pkt, truePos, obsTruePos)
%SIMULATERANGEBEARING One noisy [unit bearing; range] measurement of
%   TRUEPOS, drawn from the distribution MEASUREMENTBALLBEARING/NOISEDENSITY
%   assumes (copy of the function in run_5v5_decentralised.m).
%   Extension: optional OBSTRUEPOS = the observer's TRUE position (default
%   PKT.ObserverPosition, i.e. identical to the original). When the packet
%   carries a BELIEVED pose that differs from the truth, the raw measurement
%   is taken from the true pose (what a real sensor does).

if nargin < 3 || isempty(obsTruePos)
    obsTruePos = pkt.ObserverPosition;
end
d = truePos - obsTruePos;
r = norm(d);
u = d / r;
t = [-u(2); u(1)];

sigmaTangential2 = pkt.DetectionNoiseStd^2 + pkt.ObserverHeadingVariance ...
    + (t.' * pkt.ObserverCovariance * t) / r^2;
sigmaRange2 = pkt.RangeNoiseStd^2 + u.' * pkt.ObserverCovariance * u;

y = [u + sqrt(sigmaTangential2)*randn(2, 1); r + sqrt(sigmaRange2)*randn()];
end
