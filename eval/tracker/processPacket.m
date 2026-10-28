function [tracks, evs] = processPacket(tracks, pkt, params, truth)
%PROCESSPACKET One robot's copy of the tracker handling one received
%   packet: predict, associate (gating on the FULL innovation covariance,
%   optionally one detection per track), fuse or spawn, then the
%   expected-but-unseen deletion and duplicate-merge bookkeeping.
%   EVS is the debug event log for this packet (see MAKEEVENT).

t = pkt.time;
evs = makeEvent('x', 0, '', '', '', 0, 0, 0, false, '', 0);
evs(1) = [];

% Bring every live track to the scan time once, up front (SYSTEM.PREDICT
% is a no-op for dt == 0, so the fusion step below doesn't redo it).
for k = find([tracks.alive])
    tracks(k).system = tracks(k).system.predict(t);
end

% ---- optional ball/robot study options (all default OFF; see trackerParamsFill) ----
if isfield(params, 'ballResetN') && ~isfield(tracks, 'rej')
    tracks = addRejField(tracks);
end
if isfield(params, 'ballResetTimes') && (~isempty(params.ballResetTimes) || ~isempty(params.robotResetEvents))
    [tracks, evs] = applyExternalEvents(tracks, t, params, evs);
end

dets = pkt.detections;
nd   = numel(dets);
nt   = numel(tracks);

% Squared Mahalanobis distance of every detection to every live track of
% its class. D2S uses the full innovation covariance S = H*P*H' + R (the
% right gate); D2R uses the measurement noise R alone (what the first
% version gated on -- logged only so the two can be compared).
D2S = inf(nd, nt);
D2R = inf(nd, nt);
LDS = zeros(nd, nt);
isSelf = false(1, nt); % robot-class tracks sitting on the sender: it never detects itself
for k = find([tracks.alive] & strcmp({tracks.Class}, 'robot'))
    mu = tracks(k).system.density.mean();
    isSelf(k) = norm(mu(1:2) - pkt.ObserverPosition) < params.selfExclusion;
end
for i = 1:nd
    mScore = makeMeasurement(pkt, dets(i), params.obsCovInflate);
    for k = find([tracks.alive] & strcmp({tracks.Class}, dets(i).Class) & ~isSelf)
        [D2S(i, k), D2R(i, k), LDS(i, k)] = gateStats(mScore, tracks(k).system, dets(i).y);
    end
end

% Assignment: ASSIGN(i) = track index detection i is fused into, 0 = spawn.
assign = zeros(1, nd);
C = D2S;
if params.assocLogDet
    C = C + LDS; % -2 log N(nu; 0, S) up to a constant: prefers the tighter track at equal distance
end
C(D2S > params.gateThreshold) = inf;
if params.oneToOne
    % Greedy global nearest neighbour: repeatedly take the closest
    % remaining (detection, track) pair. Tracks are independent filters,
    % so distances computed before any fusion stay valid for the rest.
    while true
        [m, lin] = min(C(:));
        if isempty(m) || ~isfinite(m)
            break
        end
        [i, k] = ind2sub(size(C), lin);
        assign(i) = k;
        C(i, :) = inf;
        C(:, k) = inf;
    end
else
    for i = 1:nd
        [m, k] = min(C(i, :));
        if ~isempty(m) && isfinite(m)
            assign(i) = k;
        end
    end
end

% Option C (pure tracking, no external info): if ball detections keep failing
% the gate of every live ball track, the old ball track is wrong -> end it
% and let the failing detection start a clean one.
if isfield(params, 'ballResetN') && (isfinite(params.ballResetN) || isfinite(params.ballResetD2) || isfinite(params.ballResetVmax))
    bT = find([tracks.alive] & strcmp({tracks.Class}, 'ball'));
    bD = find(strcmp({dets.Class}, 'ball'));
    bT = bT(bT <= nt);
    if ~isempty(bT) && ~isempty(bD)
        if any(assign(bD) > 0)
            for k = bT, tracks(k).rej = 0; end
        else
            immediate = min(D2S(bD, bT), [], 'all') > params.ballResetD2;
            for k = bT
                mu = tracks(k).system.density.mean();
                dtk = max(t - tracks(k).queue(end).time, 0.05);
                for i = bD
                    yy = dets(i).y(:); zp = pkt.ObserverPosition(:) + yy(3) * yy(1:2) / norm(yy(1:2));
                    if norm(zp - mu(1:2)) / dtk > params.ballResetVmax, immediate = true; end
                end
            end
            for k = bT, tracks(k).rej = tracks(k).rej + 1; end
            if immediate || tracks(bT(1)).rej >= params.ballResetN
                for k = bT
                    [tracks(k), evs] = killTrack(tracks(k), tracks, t, 'reset', evs);
                end
            end
        end
    end
end

seen = false(1, nt);
took = zeros(1, nd); % index of the track each detection ended up in
for i = 1:nd
    d = dets(i);
    if assign(i) == 0 && d.y(3) < params.minSpawnRange && strcmp(d.Class, 'robot')
        continue % too close to the sender to spawn from: range/bearing there are dominated by the observer's own pose error
    end
    pendingSpawn = [];
    if assign(i) == 0 && isfield(params, 'spawnMaxRangeStd') && spawnPolicyApplies(d, params)
        if isPoorDetection(d, pkt, params)
            % Poorly conditioned detection (huge range/position uncertainty): never
            % spawns a track by itself. Optionally park it as a pending hypothesis
            % and confirm it if another robot's detection corroborates it.
            if params.spawnPending
                [tracks, pendingSpawn] = tryPending(tracks, pkt, d, params, truth, D2S(i, :), D2R(i, :));
            end
            if isempty(pendingSpawn), continue, end
        elseif strcmp(d.Class, 'ball') && isfinite(params.ballReplaceStd)
            % a well-conditioned ball detection replaces a ball track with a huge covariance
            for kk = find([tracks.alive] & strcmp({tracks.Class}, 'ball'))
                Pk = tracks(kk).system.density.cov();
                if sqrt(max(eig(Pk(1:2, 1:2)))) > params.ballReplaceStd
                    [tracks(kk), evs] = killTrack(tracks(kk), tracks, t, 'replaced', evs);
                end
            end
        end
    end
    if ~isempty(pendingSpawn)
        % confirmed pending hypothesis: spawn at the ray intersection, fuse the old and the new detection
        [newTrack, ev] = spawnTrack(tracks, pendingSpawn.pkt, pendingSpawn.dMod, params, truth, inf(1, nt), inf(1, nt));
        newTrack = fuseDetection(newTrack, pendingSpawn.pkt, pendingSpawn.d, 0, 0, truth, params.obsCovInflate);
        [newTrack, ~, failed] = fuseDetection(newTrack, pkt, d, 0, 0, truth, params.obsCovInflate);
        tracks(end + 1) = newTrack;
        k = numel(tracks);
    elseif assign(i) == 0
        [newTrack, ev] = spawnTrack(tracks, pkt, d, params, truth, D2S(i, :), D2R(i, :));
        % Fuse the detection into the vague prior right away (so the new
        % track has a first update and a history to plot), but do not log
        % it as an association -- it would pollute the d2 calibration.
        [newTrack, ~, failed] = fuseDetection(newTrack, pkt, d, 0, 0, truth, params.obsCovInflate);
        tracks(end + 1) = newTrack; %#ok<AGROW>
        k = numel(tracks);
        % Spawn event: aux = [position std after the first fusion, its error vs the truth]
        Pn = tracks(k).system.density.cov();
        mn = tracks(k).system.density.mean();
        xT = trueStateAt(d.TrueLabel, pkt.time, truth);
        ev.life = sqrt(max(eig(Pn(1:2, 1:2))));
        ev.aux  = norm(mn(1:2) - xT(1:2));
    else
        k = assign(i);
        [tracks(k), ev, failed] = fuseDetection(tracks(k), pkt, d, D2S(i, k), D2R(i, k), truth, params.obsCovInflate);
    end
    if failed
        evs = [evs, makeEvent('fusefail', pkt.time, tracks(k).Name, tracks(k).TrueLabel, d.TrueLabel, 0, 0, 0, false, '', 0)]; %#ok<AGROW>
    end
    took(i) = k;
    seen(k) = true; % a spawn grows TRACKS, so SEEN may need to grow too
    evs = [evs, ev]; %#ok<AGROW>
end
seen(end + 1:numel(tracks)) = false;

% Cannot-link: one sensor scan gives at most one detection per real
% object, so any two tracks that took different detections from the SAME
% scan are different objects and must never be merged later (this is
% what stops two robots that walk past each other being collapsed).
for a = 1:nd
    for b = a + 1:nd
        if took(a) > 0 && took(b) > 0 && took(a) ~= took(b)
            tracks(took(a)).distinct = union(tracks(took(a)).distinct, took(b));
            tracks(took(b)).distinct = union(tracks(took(b)).distinct, took(a));
        end
    end
end

% Deletion: only tracks that SHOULD have been in this scan but weren't
% fused count a miss; a track merely out of view is left alone.
for k = find([tracks.alive] & ~seen)
    mu = tracks(k).system.density.mean();
    isSender = strcmp(tracks(k).Class, 'robot') && norm(mu(1:2) - pkt.ObserverPosition) < params.selfExclusion;
    if ~isSender && expectedVisible(mu, pkt, params)
        tracks(k).misses = tracks(k).misses + 1;
        if tracks(k).misses >= params.maxMisses
            tracks(k).alive        = false;
            tracks(k).deletedAt    = t;
            tracks(k).DeleteReason = 'missed';
            lastForLabel = ~any(strcmp({tracks.TrueLabel}, tracks(k).TrueLabel) & [tracks.alive]);
            evs = [evs, makeEvent('delete', t, tracks(k).Name, tracks(k).TrueLabel, '', 0, 0, ...
                numel(tracks(k).queue), lastForLabel, 'missed', t - tracks(k).SpawnTime)]; %#ok<AGROW>
        end
    end
end

% Lost tracks: a track whose position uncertainty has grown beyond
% LOSTSTD (or that has gone LOSTTIME s without a fused detection) carries
% no information any more and is deleted (it would otherwise linger as a
% stale ghost/duplicate once its robot walks off).
for k = find([tracks.alive])
    if tracks(k).alive && strcmp(tracks(k).Class, 'robot') && ~seen(k)
        Pk = tracks(k).system.density.cov();
        lastT = tracks(k).queue(end).time;
        if sqrt(max(eig(Pk(1:2, 1:2)))) > params.lostStd || t - lastT > params.lostTime
            tracks(k).alive        = false;
            tracks(k).deletedAt    = t;
            tracks(k).DeleteReason = 'lost';
            lastForLabel = ~any(strcmp({tracks.TrueLabel}, tracks(k).TrueLabel) & [tracks.alive]);
            evs = [evs, makeEvent('delete', t, tracks(k).Name, tracks(k).TrueLabel, '', 0, 0, ...
                numel(tracks(k).queue), lastForLabel, 'lost', t - tracks(k).SpawnTime)]; %#ok<AGROW>
        end
    end
end

if ~isnan(params.mergeThreshold)
    [tracks, mevs] = mergeDuplicates(tracks, t, params);
    evs = [evs, mevs];
end

% Option B: lost ball tracks (same rule as robots, ball-specific thresholds).
if isfield(params, 'ballLostStd')
    for k = find([tracks.alive] & strcmp({tracks.Class}, 'ball') & ~seen(1:numel(tracks)))
        Pk = tracks(k).system.density.cov();
        if sqrt(max(eig(Pk(1:2, 1:2)))) > params.ballLostStd || t - tracks(k).queue(end).time > params.ballLostTime
            [tracks(k), evs] = killTrack(tracks(k), tracks, t, 'lost', evs);
        end
    end
end

% Option A: there is only one ball -> keep the live ball track with the
% smallest position-covariance determinant (most information now).
if isfield(params, 'ballSingle') && params.ballSingle
    bl = find([tracks.alive] & strcmp({tracks.Class}, 'ball'));
    if numel(bl) > 1
        dd = zeros(1, numel(bl));
        for a = 1:numel(bl)
            Pk = tracks(bl(a)).system.density.cov();
            dd(a) = det(Pk(1:2, 1:2));
        end
        [~, ib] = min(dd);
        for a = setdiff(1:numel(bl), ib)
            [tracks(bl(a)), evs] = killTrack(tracks(bl(a)), tracks, t, 'single', evs);
        end
    end
end
end

function [trk, evs] = killTrack(trk, tracks, t, reason, evs, tDel)
if nargin < 6, tDel = t; end
trk.alive = false; trk.deletedAt = tDel; trk.DeleteReason = reason;
lastForLabel = ~any(strcmp({tracks.TrueLabel}, trk.TrueLabel) & [tracks.alive]);
evs = [evs, makeEvent('delete', t, trk.Name, trk.TrueLabel, '', 0, 0, numel(trk.queue), lastForLabel, reason, t - trk.SpawnTime)];
end

function tracks = addRejField(tracks)
if isempty(tracks)
    tracks(1).rej = 0; tracks(1) = [];
else
    for k = 1:numel(tracks), tracks(k).rej = 0; end
end
end

function [tracks, evs] = applyExternalEvents(tracks, t, params, evs)
%APPLYEXTERNALEVENTS Referee-style broadcast (EXTERNAL info, options D / robot
%   resets). Events with te <= t that happened after a track was spawned end it.
%   Ball: every live ball track spawned before te is ended (deletedAt = te).
%   Robot: events [te x y]; live robot tracks spawned before te whose mean is within
%   robotResetGate m of the from-position OR inside its 95% region of it
%   are ended (ambiguous -> delete; the filter re-acquires).
for te = params.ballResetTimes(:)'
    if te > t, continue, end
    for k = find([tracks.alive] & strcmp({tracks.Class}, 'ball') & [tracks.SpawnTime] < te)
        [tracks(k), evs] = killTrack(tracks(k), tracks, t, 'event', evs, te);
    end
end
E = params.robotResetEvents;
for i = 1:size(E, 1)
    te = E(i, 1);
    if te > t, continue, end
    for k = find([tracks.alive] & strcmp({tracks.Class}, 'robot') & [tracks.SpawnTime] < te)
        m = tracks(k).system.density.mean(); P = tracks(k).system.density.cov();
        d = m(1:2) - E(i, 2:3).';
        if norm(d) <= params.robotResetGate || d.' * (P(1:2, 1:2) \ d) <= 5.9915
            [tracks(k), evs] = killTrack(tracks(k), tracks, t, 'event', evs, te);
        end
    end
end
end

function tf = spawnPolicyApplies(d, params)
tf = strcmp(d.Class, 'ball') || params.spawnPolicyRobots;
end

function [poor, rs, area] = isPoorDetection(d, pkt, params)
%ISPOORDETECTION Range std (per-detection if available) or implied position
%   covariance area (pi*sigma_r*sigma_t) above the spawn thresholds.
rs = pkt.RangeNoiseStd;
if isfield(d, 'LastDetection') && isstruct(d.LastDetection) && isfield(d.LastDetection, 'RangeStd') && isfinite(d.LastDetection.RangeStd)
    rs = d.LastDetection.RangeStd;
end
st = d.y(3) * hypot(pkt.DetectionNoiseStd, sqrt(pkt.ObserverHeadingVariance));
area = pi * rs * st;
poor = rs > params.spawnMaxRangeStd || area > params.spawnMaxArea;
end

function [tracks, ps] = tryPending(tracks, pkt, d, params, truth, d2S, d2R)
%TRYPENDING Corroborate detection D against pending hypotheses of the same
%   class from a DIFFERENT robot within spawnPendWindow s (rays must intersect
%   in front of both observers, at a decent angle, with ranges consistent
%   within 3 sigma + 0.5 m). Returns the confirmed pending info PS (empty if
%   none) -- otherwise D is parked as a new pending hypothesis.
t = pkt.time; ps = [];
pend = find(strcmp({tracks.DeleteReason}, 'pending') & strcmp({tracks.Class}, d.Class));
for j = pend
    q = tracks(j).distinct;
    if t - q.pkt.time > params.spawnPendWindow
        tracks(j).DeleteReason = 'expired';
    end
end
pend = find(strcmp({tracks.DeleteReason}, 'pending') & strcmp({tracks.Class}, d.Class));
[~, rs2] = isPoorDetection(d, pkt, params);
u2 = d.y(1:2); u2 = u2(:) / norm(u2); o2 = pkt.ObserverPosition(:);
for j = pend
    q = tracks(j).distinct;
    if q.pkt.RobotID == pkt.RobotID, continue, end
    [~, rs1] = isPoorDetection(q.d, q.pkt, params);
    u1 = q.d.y(1:2); u1 = u1(:) / norm(u1); o1 = q.pkt.ObserverPosition(:);
    A = [u1, -u2];
    if abs(det(A)) < sind(10), continue, end
    sol = A \ (o2 - o1);
    if any(sol <= 0), continue, end
    if abs(sol(1) - q.d.y(3)) > 3*rs1 + 0.5 || abs(sol(2) - d.y(3)) > 3*rs2 + 0.5, continue, end
    dMod = q.d; dMod.y(3) = sol(1);
    tracks(j).DeleteReason = 'confirmed';
    ps = struct('pkt', q.pkt, 'd', q.d, 'dMod', dMod);
    return
end
% park as pending
pk = pkt; pk.detections = [];
[nt_, ~] = spawnTrack(tracks, pkt, d, params, truth, d2S, d2R);
nt_.alive = false; nt_.DeleteReason = 'pending'; nt_.deletedAt = NaN;
nt_.distinct = struct('pkt', pk, 'd', d);
if isfield(tracks, 'rej') && ~isfield(nt_, 'rej'), nt_.rej = 0; end
tracks(end + 1) = nt_;
end
