function m = makeMeasurement(pkt, d, infl)
%MAKEMEASUREMENT The MEASUREMENTBALLBEARING for detection D of packet PKT.
if nargin < 3, infl = [1 1]; end % [robot ball] covariance inflation
if strcmp(d.Class, 'ball'), infl = infl(end); else, infl = infl(1); end
m = MeasurementBallBearing();
m.time                    = pkt.time;
m.ObserverPosition        = pkt.ObserverPosition;
m.ObserverCovariance      = pkt.ObserverCovariance * infl; % INFL > 1 covers the temporally correlated part of the sender's pose error
m.DetectionNoiseStd       = pkt.DetectionNoiseStd;
m.RangeNoiseStd           = pkt.RangeNoiseStd;
m.ObserverHeadingVariance = pkt.ObserverHeadingVariance;
m.RobotID                 = pkt.RobotID;
if isfield(d, 'LastDetection') && isstruct(d.LastDetection) && isfield(d.LastDetection, 'RangeStd') ...
        && isfinite(d.LastDetection.RangeStd)   % per-detection range std (camera model)
    m.RangeNoiseStd = d.LastDetection.RangeStd;
end
m.y                       = d.y;
m.verbosity               = 0;
m.updateMethod            = 'BFGSTrustSqrt';
end
