function [track, ev, failed] = fuseDetection(track, pkt, d, d2S, d2R, truth, obsCovInflate)
if nargin < 7, obsCovInflate = 1; end
%FUSEDETECTION Fuse detection D into TRACK (already predicted to the
%   scan time) and return the debug event.

m = makeMeasurement(pkt, d, obsCovInflate);
m.AlreadyDetected = true;
m.LastDetection   = d.LastDetection;
m.TargetName      = track.Name;
m.needToSimulate  = false;

track.system.x_sim = trueStateAt(track.TrueLabel, pkt.time, truth); % ground truth, for plotting only
track.misses = 0;

% MEASUREMENTBALLBEARING/UPDATE already catches and warns on optimiser
% non-convergence itself (leaving the density at its predict-only state).
lastwarn('');
[m, track.system] = m.process(track.system);
[~, warnId] = lastwarn;
failed = strcmp(warnId, 'MeasurementBallBearing:updateFailed');
track.queue(end + 1) = m;

ev = makeEvent('assoc', pkt.time, track.Name, track.TrueLabel, d.TrueLabel, d2S, d2R, ...
    numel(track.queue), strcmp(track.TrueLabel, d.TrueLabel), '', 0);
end
