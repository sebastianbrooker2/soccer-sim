function copies = runRawMeasurement(packets, params)
%RUNRAWMEASUREMENT "Raw measurement" behaviour: NO tracking at all.
%   copies = runRawMeasurement(packets, params) -> struct array, copies(r) for observer r.
%   No filter, no prediction, no association memory, no sharing: each detection of each packet
%   of robot r is turned into an instantaneous Cartesian position estimate straight from the
%   measurement model; out of sight (or not detected) = no estimate.
%
%   Per detection y = [ux; uy; range] (unit bearing in the field frame + range), packet fields
%   as the SRIF's MeasurementBallBearing sees them (eval/tracker/makeMeasurement.m):
%     mean = ObserverPosition + range * u/|u|
%     tangential variance (rad^2) sT2 = DetectionNoiseStd^2 + ObserverHeadingVariance + t'*C*t/range^2
%     range variance             sR2 = RangeStd^2 + u'*C*u      (RangeStd = LastDetection.RangeStd if
%                                      finite else packet RangeNoiseStd; C = ObserverCovariance)
%     cov = range^2*sT2 * t*t' + sR2 * u*u'      (== the bearing/range noiseDensity propagated to a point)
%   NO obsCovInflate (that is a tracker-side prior for temporally correlated pose error) and no
%   process noise / prior. Packets already carry the YOLO confidence thresholds and teammate drop;
%   false positives are kept (a trusting consumer would use them).
%   copies(r).raw: .t (packet times, sorted), .cls{i}/.m{i} (2xn means)/.C{i} (2x2xn)/.lbl{i}
%   (diagnostic TrueLabel, never used). Read with rawEstimates (stable API).
if nargin < 2, params = struct(); end %#ok<INUSD>
rid = arrayfun(@(p) p.RobotID, packets);
nObs = max(rid);
copies = struct('robot', cell(1, nObs), 'tracks', [], 'raw', []);
for r = 1:nObs
    P = packets(rid == r); [~, o] = sort([P.time]); P = P(o);
    n = numel(P); raw = struct('t', [P.time], 'cls', {cell(n,1)}, 'm', {cell(n,1)}, 'C', {cell(n,1)}, 'lbl', {cell(n,1)});
    for i = 1:n
        pk = P(i); D = pk.detections; k = numel(D);
        m = zeros(2,k); C = zeros(2,2,k); cls = cell(1,k); lbl = cell(1,k);
        for j = 1:k
            d = D(j); y = d.y(:); r_ = y(3); u = y(1:2)/norm(y(1:2)); t = [-u(2); u(1)];
            rs = pk.RangeNoiseStd;
            if isfield(d,'LastDetection') && isstruct(d.LastDetection) && isfield(d.LastDetection,'RangeStd') && isfinite(d.LastDetection.RangeStd)
                rs = d.LastDetection.RangeStd;
            end
            Co = pk.ObserverCovariance;
            sT2 = pk.DetectionNoiseStd^2 + pk.ObserverHeadingVariance + (t.'*Co*t)/r_^2;
            sR2 = rs^2 + u.'*Co*u;
            Cj = r_^2*sT2*(t*t.') + sR2*(u*u.');
            m(:,j) = pk.ObserverPosition(:) + r_*u; C(:,:,j) = (Cj+Cj.')/2;
            if strcmp(d.Class,'ball'), cls{j} = 'ball'; else, cls{j} = 'robot'; end
            if isfield(d,'TrueLabel'), lbl{j} = d.TrueLabel; else, lbl{j} = ''; end
        end
        raw.cls{i} = cls; raw.m{i} = m; raw.C{i} = C; raw.lbl{i} = lbl;
    end
    copies(r).robot = r; copies(r).raw = raw; copies(r).tracks = struct([]);
end
end
