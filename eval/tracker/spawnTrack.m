function [track, ev] = spawnTrack(tracks, pkt, d, params, truth, d2Srow, d2Rrow)
%SPAWNTRACK New track from one unassociated range-bearing detection:
%   the prior mean is the measured range along the measured bearing, but
%   with a deliberately vague covariance (SPAWNPOSSTD position, velocity
%   unknown); the caller then fuses the detection into it. The
%   process model comes from the detector's class. The debug event says
%   whether this is the first track ever for that real object in this
%   copy ('first'), a replacement after earlier ones died ('respawn'), or
%   a 'duplicate' of a track for it that is still live -- and how far
%   (d2) the closest such live track was.

u = d.y(1:2) / norm(d.y(1:2));
r = d.y(3);

P0   = blkdiag(params.spawnPosStd^2*eye(2), 2.5^2*eye(2));
% Start the vague prior 1 m SHORT of the measured range, not on it. Exactly
% on it the likelihood and prior gradients are both zero, BFGSTrustSqrt takes
% no step, and its sqrt-Hessian (the posterior covariance) is returned
% unchanged -- i.e. the track would stay at the prior's 3 m std after its
% first fusion. Off the optimum it takes real steps and builds the Hessian.
% The resulting pull towards the prior is ~0.03 m.
mu0  = [pkt.ObserverPosition + max(r - 1, 0.5)*u; 0; 0];

if isfield(params, 'spawnAnalytic') && ((params.spawnAnalytic && ~strcmp(d.Class, 'ball')) || (strcmp(d.Class, 'ball') && params.spawnAnalyticBall))
    % Optional (default off): analytic Cartesian prior from the measurement
    % (inflated so the immediate re-fusion by the caller is not over-confident).
    tv = [-u(2); u(1)];
    C  = pkt.ObserverCovariance;
    sr2 = pkt.RangeNoiseStd^2 + u.'*C*u;
    st2 = r^2*(pkt.DetectionNoiseStd^2 + pkt.ObserverHeadingVariance) + tv.'*C*tv;
    infl = params.spawnInflate;
    Pp = infl*(sr2*(u*u.') + st2*(tv*tv.'));
    if strcmp(d.Class, 'ball'), vs = 2.5; else, vs = params.spawnVelStd; end
    P0 = blkdiag(Pp, vs^2*eye(2));
    mu0 = [pkt.ObserverPosition + r*u; 0; 0];
end

sys = SystemBall();
sys.time = pkt.time;
if strcmp(d.Class, 'ball')
    sys.Kappa         = params.kappaBall;
    sys.AccelNoiseStd = params.ballAccelStd;
else
    sys.Kappa         = 0;
    sys.AccelNoiseStd = params.robotAccelStd;
end
sys.density = GaussianInfo.fromMoment(mu0, P0);
sys.x_sim   = trueStateAt(d.TrueLabel, pkt.time, truth); % ground truth, for the pre-first-event display only

track.Name          = sprintf('trk%d', numel(tracks) + 1);
track.Class         = d.Class;
track.TrueLabel     = d.TrueLabel;
track.system        = sys;
track.initialSystem = sys;
track.queue         = MeasurementBallBearing.empty;
track.misses        = 0;
track.alive         = true;
track.deletedAt     = nan;
track.SpawnTime     = pkt.time;
track.DeleteReason  = '';
track.distinct      = [];
if isfield(params, 'ballResetN'), track.rej = 0; end

sameLabel = find(strcmp({tracks.TrueLabel}, d.TrueLabel));
liveSame  = sameLabel([tracks(sameLabel).alive]);
liveSame  = liveSame(liveSame <= numel(d2Srow)); % only tracks that existed when D2 was computed
if isempty(sameLabel)
    note = 'first';
elseif isempty([tracks(sameLabel).alive]) || ~any([tracks(sameLabel).alive])
    note = 'respawn';
else
    note = 'duplicate';
end
if isempty(liveSame)
    bestS = inf;
    bestR = inf;
else
    bestS = min(d2Srow(liveSame));
    bestR = min(d2Rrow(liveSame));
end
ev = makeEvent('spawn', pkt.time, track.Name, d.TrueLabel, d.TrueLabel, bestS, bestR, ...
    numel(liveSame), false, note, 0);
end
