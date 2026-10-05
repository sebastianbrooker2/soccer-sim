% RUN_5V5_DECENTRALISED 5v5, decentralised: every robot keeps its OWN copy
%   of the shared set of tracks and fuses range-bearing detections from
%   every robot it hears from -- including its own -- into that copy.
%   Nothing is global: there is no central filter. Same mathematics
%   whether a detection is fused by the robot that made it or by a
%   teammate that received it over the network.
%
%   What is modelled, and where it differs from the earlier 5v5 scripts
%   (RUN_5V5_STATIONARY_ROBOTS / RUN_5V5_DATA_ASSOCIATION):
%
%   1. DECENTRALISED. Each of our 5 robots runs its own data association
%      and its own tracks (COPIES(r).tracks). Each robot broadcasts one
%      PACKET per scan: its pose/covariance, its FOV/range/look
%      direction, and the list of detections (possibly empty). Every
%      packet reaches each OTHER robot independently with probability
%      1 - LINKDROPPROB (the sender always "receives" its own packet),
%      so copies can disagree: a robot that missed a packet has less
%      information, and may even have spawned or deleted different
%      tracks. With LINKDROPPROB = 0 all copies come out identical.
%      Latency / out-of-order arrival is NOT modelled (packets are
%      fused in time order); only loss is.
%
%   2. RANGE-BEARING MEASUREMENT. Each detection is y = [unit bearing
%      vector; range] (3x1, field frame) with a fixed range std per
%      robot (RANGEnoiseStd below) -- MEASUREMENTBALLBEARING with
%      RangeNoiseStd set (see its PREDICT/NOISEDENSITY). One robot now
%      localises a target on its own; the bearing still only constrains
%      the target across the line of sight, the range along it.
%
%   3. BALL PROCESS MODEL. The ball uses rolling-friction decay
%      (SYSTEMBALL.Kappa > 0: dv/dt = -kappa*v), with a larger fixed
%      acceleration noise to absorb kicks. Robots (opponents and
%      teammates) stay constant-velocity with their own noise. The
%      detector reports a CLASS ('ball' or 'robot') but not an identity,
%      so a new track takes the right process model from its first
%      detection's class, and detections only gate against tracks of
%      the same class.
%
%   4. TRACK DELETION ONLY WHEN EXPECTED-BUT-UNSEEN. Being out of view
%      is not evidence a target is gone. After fusing a packet, a copy
%      checks each live track it did not just fuse: if that track's
%      estimated position was inside the sender's scan (within
%      EXPECTEDMARGIN of the FOV half-angle and of the range, and not
%      the sender itself) it counts a MISS; a track that was not
%      expected in that scan is left alone. A fused detection resets
%      the count, and MAXMISSES consecutive misses delete the track.
%      (Several are needed because the detector's confidence jitter
%      makes single misses normal. False-positive detections are not
%      simulated, so deletion here mostly clears fragmented/duplicate
%      tracks.)
%
%   Everything else is the same as RUN_5V5_DATA_ASSOCIATION: unlabelled
%   detections, nearest-neighbour gating in Mahalanobis distance, 5
%   stationary robots with panning heads, scripted ball kicks, random-
%   walking opponents, sqrt-information BFGS fusion. Robots are still
%   stationary with a known field-frame position (their world-to-field
%   transform is not modelled; ObserverPosition/Covariance are simply
%   given in the field frame). TrueLabel is stapled on only so we can
%   see (and score) what a track locked onto; the associator never reads
%   it.
%
%   NOT RUN YET in the environment this was written in (no MATLAB there):
%   expect to fix typos/tuning on first run. The process-noise and
%   deletion parameters below are untuned first guesses.

clc;
clear all;

verbosity = 0;  % 0: silent per event (5 copies x thousands of events), 1+: print every event

% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Parameters
linkDropProb   = 0.20;  % probability a packet is lost on a given sender -> receiver link
gateSigma      = 3;     % association gate, in sigmas (chi-square region of the same probability mass, on the full innovation covariance S = H*P*H' + R)
oneToOne       = true;  % within one scan, a track may take at most one detection (greedy global nearest neighbour); false = each detection independently takes its nearest gated track
mergeSigma     = 3;     % live same-class tracks whose position estimates agree within this many sigmas (combined covariance) are duplicates: the one with fewer fused detections is dropped. NaN = never merge
maxMisses      = 5;     % consecutive expected-but-unseen scans before a track is deleted
expectedMargin = 0.8;   % fraction of FOV half-angle / range inside which a track counts as "expected" in a scan (the detector's confidence fades toward the edge)
selfExclusion  = 0.5;   % m, tracks this close to the sender are the sender itself, never expected
kappaBall      = 0.2;   % 1/s, ball rolling-friction velocity decay rate
ballAccelStd   = 2.5;   % m/s^2 per sqrt(Hz), fixed process noise for the ball (absorbs kicks; 0.5 lost the ball track after every kick)
robotAccelStd  = 0.5;   % same, for robots (the opponents' random walk turns at up to 45 deg/s at ~0.8 m/s, ~0.6 m/s^2, plus speed noise 0.5)
spawnPosStd    = 3;     % m, deliberately vague isotropic position prior for a new track; the first detection is then FUSED into it (no double counting)
T_total        = 15;    % seconds
viewRobot      = 1;     % whose copy the figures and step-through tool show
debugLog       = true;  % print the spawn/delete/merge event log of the viewRobot copy
debugMaxLines  = 120;   % ...truncated to this many lines

%% Stationary sensing robots -- same 5v5 formation as RUN_5V5_DATA_ASSOCIATION
% (our five on x<0 facing forward, head-panning +-80 degrees, Range scaled
% up 1.8x), plus a fixed RangeNoiseStd per robot.
clear robots

RANGE_SCALE = 1.8;

robots(1).Position          = [-8; -4];
robots(1).Covariance        = diag([0.02, 0.02]);
robots(1).DetectionNoiseStd = deg2rad(0.75);
robots(1).RangeNoiseStd     = 0.30;
robots(1).Rate               = 2;
robots(1).Heading           = 0;
robots(1).FOV               = deg2rad(100);
robots(1).Range             = RANGE_SCALE * 15;
robots(1).GazeAmplitude      = deg2rad(80);
robots(1).GazeAngularSpeed   = 2*pi/6;   % 6 s per full left-right-left cycle
robots(1).GazePhase0         = 0;

robots(2).Position          = [-8; 4];
robots(2).Covariance        = diag([0.05, 0.05]);
robots(2).DetectionNoiseStd = deg2rad(1.5);
robots(2).RangeNoiseStd     = 0.50;
robots(2).Rate               = 3;
robots(2).Heading           = 0;
robots(2).FOV               = deg2rad(130);
robots(2).Range             = RANGE_SCALE * 10;
robots(2).GazeAmplitude      = deg2rad(80);
robots(2).GazeAngularSpeed   = 2*pi/6;
robots(2).GazePhase0         = pi/2;

robots(3).Position          = [-3; -6];
robots(3).Covariance        = diag([0.01, 0.01]);
robots(3).DetectionNoiseStd = deg2rad(0.4);
robots(3).RangeNoiseStd     = 0.20;
robots(3).Rate               = 5;
robots(3).Heading           = 0;
robots(3).FOV               = deg2rad(110);
robots(3).Range             = RANGE_SCALE * 10;
robots(3).GazeAmplitude      = deg2rad(80);
robots(3).GazeAngularSpeed   = 2*pi/6;
robots(3).GazePhase0         = pi;

robots(4).Position          = [-3; 6];
robots(4).Covariance        = diag([0.03, 0.03]);
robots(4).DetectionNoiseStd = deg2rad(1.25);
robots(4).RangeNoiseStd     = 0.40;
robots(4).Rate               = 4;
robots(4).Heading           = 0;
robots(4).FOV               = deg2rad(140);
robots(4).Range             = RANGE_SCALE * 10;
robots(4).GazeAmplitude      = deg2rad(80);
robots(4).GazeAngularSpeed   = 2*pi/6;
robots(4).GazePhase0         = 3*pi/2;

robots(5).Position          = [-5.5; 0];
robots(5).Covariance        = diag([0.02, 0.02]);
robots(5).DetectionNoiseStd = deg2rad(1.0);
robots(5).RangeNoiseStd     = 0.35;
robots(5).Rate               = 3;
robots(5).Heading           = 0;
robots(5).FOV               = deg2rad(120);
robots(5).Range             = RANGE_SCALE * 12;
robots(5).GazeAmplitude      = deg2rad(80);
robots(5).GazeAngularSpeed   = 2*pi/6;
robots(5).GazePhase0         = 8*pi/5;

numTeam = numel(robots);

%% Ground truth for everything on the field, precomputed on a dense grid
% Only used to (a) decide whether a detection happens, (b) draw its noisy
% range-bearing, and (c) bookkeeping/plots. Not available to the filters.

cfg   = fieldConfig();
dtSim = 0.02;
tGrid = 0:dtSim:T_total;
Nt    = numel(tGrid);

% Ball: scripted kicks, integrated with the SAME exponential friction
% decay the ball filter assumes (exactly, per step).
kickTimes      = [1.5, 3.5, 6.5, 10];
kickVelocities = [-0.75, -3.875; ...
                   -3.917,  3.375; ...
                    5.667,  1.43; ...
                   -1.9,   -1.38];

ballPos = zeros(2, Nt);
ballVel = zeros(2, Nt);
bx = [0; 0];
bv = [1; 0.5];
nextKick = 1;
decay = exp(-kappaBall*dtSim);
for kk = 1:Nt
    ballPos(:, kk) = bx;
    ballVel(:, kk) = bv;
    if kk < Nt
        bx = bx + bv*(1 - decay)/kappaBall;
        bv = bv*decay;
        tNext = tGrid(kk + 1);
        while nextKick <= size(kickVelocities, 1) && tNext >= kickTimes(nextKick)
            bv = bv + kickVelocities(nextKick, :).';
            nextKick = nextKick + 1;
        end
    end
end

% Opposition: same 5-robot smooth random walk as the earlier 5v5 scripts.
oppStarts(1) = struct('Position', [4, -5], 'Heading', deg2rad(60));
oppStarts(2) = struct('Position', [5, 5],  'Heading', deg2rad(-120));
oppStarts(3) = struct('Position', [8, 0],  'Heading', deg2rad(180));
oppStarts(4) = struct('Position', [3, -2], 'Heading', deg2rad(90));
oppStarts(5) = struct('Position', [3, 2],  'Heading', deg2rad(-90));
numOpp = numel(oppStarts);

s = rng;
rng(42);

oppPos = cell(1, numOpp);
oppVel = cell(1, numOpp);
for o = 1:numOpp
    player = createPlayer(oppStarts(o).Position, oppStarts(o).Heading, [0.1 0; 0 0.1]);
    posHist = zeros(2, Nt);
    velHist = zeros(2, Nt);
    for kk = 1:Nt
        posHist(:, kk) = player.Position(:);
        velHist(:, kk) = player.Speed * [cos(player.Heading); sin(player.Heading)];
        if kk < Nt
            player = stepPlayerWalk(player, cfg, dtSim);
        end
    end
    oppPos{o} = posHist;
    oppVel{o} = velHist;
end

truth.dtSim   = dtSim;
truth.Nt      = Nt;
truth.ballPos = ballPos;
truth.ballVel = ballVel;
truth.oppPos  = oppPos;
truth.oppVel  = oppVel;
truth.robots  = robots;

%% Packets: one per (robot, scan time), each carrying that scan's detections
% A scan sees every real object that clears COMPUTEBALLDETECTION's
% FOV/range/confidence gate. The packet is sent even when it detected
% nothing, because "I looked here and saw nothing" is what lets
% receivers decide a track has gone (see header, point 4).

packets = struct('time', {}, 'RobotID', {}, 'ObserverPosition', {}, 'ObserverCovariance', {}, ...
    'DetectionNoiseStd', {}, 'RangeNoiseStd', {}, 'ObserverHeadingVariance', {}, ...
    'LookHeading', {}, 'FOV', {}, 'Range', {}, 'detections', {});

for r = 1:numTeam
    for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
        kk = min(max(round(t/dtSim) + 1, 1), Nt);

        candidates = struct('Label', {}, 'Class', {}, 'Position', {});
        candidates(end + 1) = struct('Label', 'ball', 'Class', 'ball', 'Position', ballPos(:, kk)); %#ok<SAGROW>
        for o = 1:numOpp
            candidates(end + 1) = struct('Label', sprintf('opp%d', o), 'Class', 'robot', 'Position', oppPos{o}(:, kk)); %#ok<SAGROW>
        end
        for j = 1:numTeam
            if j == r
                continue % a robot doesn't detect itself
            end
            candidates(end + 1) = struct('Label', sprintf('team%d', j), 'Class', 'robot', 'Position', robots(j).Position); %#ok<SAGROW>
        end

        gazeOffset = robots(r).GazeAmplitude * sin(robots(r).GazePhase0 + robots(r).GazeAngularSpeed*t);
        sensor.Position   = robots(r).Position.';
        sensor.Heading    = robots(r).Heading;
        sensor.GazeOffset = gazeOffset;
        sensor.FOV        = robots(r).FOV;
        sensor.Range      = robots(r).Range;

        pkt.time                    = t;
        pkt.RobotID                 = r;
        pkt.ObserverPosition        = robots(r).Position;
        pkt.ObserverCovariance      = robots(r).Covariance;
        pkt.DetectionNoiseStd       = robots(r).DetectionNoiseStd;
        pkt.RangeNoiseStd           = robots(r).RangeNoiseStd;
        pkt.ObserverHeadingVariance = 0;
        pkt.LookHeading             = robots(r).Heading + gazeOffset;
        pkt.FOV                     = robots(r).FOV;
        pkt.Range                   = robots(r).Range;
        pkt.detections = struct('y', {}, 'Class', {}, 'TrueLabel', {}, 'LastDetection', {});

        for c = 1:numel(candidates)
            obj_.Position = candidates(c).Position.';
            det = computeBallDetection(sensor, obj_);
            if ~det.Detected
                continue
            end
            y = simulateRangeBearing(pkt, candidates(c).Position);
            pkt.detections(end + 1) = struct('y', y, 'Class', candidates(c).Class, ...
                'TrueLabel', candidates(c).Label, 'LastDetection', det); %#ok<SAGROW>
        end
        packets(end + 1) = pkt; %#ok<SAGROW>
    end
end

[~, order] = sort([packets.time]);
packets = packets(order);
numPackets = numel(packets);

% Network: independent loss on every sender -> receiver link. Drawn once,
% up front, so a rerun with the same seed loses the same packets.
delivered = rand(numTeam, numPackets) >= linkDropProb;
for p = 1:numPackets
    delivered(packets(p).RobotID, p) = true; % a robot always has its own packet
end

rng(s);

fprintf(1, 'Generated %d packets (%d detections) from %d robots; %.0f%% of links drop.\n', ...
    numPackets, sum(arrayfun(@(q) numel(q.detections), packets)), numTeam, 100*linkDropProb);

%% Every robot runs its own copy of the whole tracker

params.gateThreshold  = chi2inv(2*normcdf(gateSigma) - 1, 3);  % on the 3x1 range-bearing innovation
params.oneToOne       = oneToOne;
params.spawnPosStd    = spawnPosStd;
params.mergeThreshold = chi2inv(2*normcdf(mergeSigma) - 1, 2);  % on the 2D position difference (NaN if mergeSigma is NaN)
params.maxMisses      = maxMisses;
params.expectedMargin = expectedMargin;
params.selfExclusion  = selfExclusion;
params.kappaBall      = kappaBall;
params.ballAccelStd   = ballAccelStd;
params.robotAccelStd  = robotAccelStd;
params.verbosity      = verbosity;

aliveHist = zeros(numTeam, numPackets); % live tracks in each copy after each packet (carried forward when that copy missed it)
copies = struct('tracks', cell(1, numTeam), 'log', cell(1, numTeam));

for r = 1:numTeam
    tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, ...
        'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
    evLog = makeEvent('x', 0, '', '', '', 0, 0, 0, false, '', 0);
    evLog(1) = [];
    numAlive = 0;
    for p = 1:numPackets
        if delivered(r, p)
            [tracks, evs] = processPacket(tracks, packets(p), params, truth);
            evLog = [evLog, evs]; %#ok<AGROW>
            numAlive = sum([tracks.alive]);
        end
        aliveHist(r, p) = numAlive;
    end
    copies(r).tracks = tracks;
    copies(r).log    = evLog;
    fprintf(1, 'Robot %d copy: %d tracks spawned, %d live, %d deleted (%d/%d packets received).\n', ...
        r, numel(tracks), sum([tracks.alive]), sum(~[tracks.alive]), sum(delivered(r, :)), numPackets);
end

%% How well do the copies agree with the truth (and each other)?
% For every real object, the live track in each copy (same class) whose
% estimated position, compared against that object's true position at
% that track's own time, is nearest -- position error in metres.
labels = [{'ball'}, arrayfun(@(o) sprintf('opp%d', o), 1:numOpp, 'UniformOutput', false), ...
    arrayfun(@(j) sprintf('team%d', j), 1:numTeam, 'UniformOutput', false)];
err = nan(numTeam, numel(labels));
for r = 1:numTeam
    tr = copies(r).tracks;
    for li = 1:numel(labels)
        best = inf;
        for k = find([tr.alive])
            xTrue = trueStateAt(labels{li}, tr(k).system.time, truth);
            mu = tr(k).system.density.mean();
            e = norm(mu(1:2) - xTrue(1:2));
            if e < best
                best = e;
            end
        end
        if isfinite(best)
            err(r, li) = best;
        end
    end
end
fprintf(1, '\nNearest live track''s position error [m] (rows = robot copies, cols = real objects):\n');
disp(array2table(err, 'VariableNames', labels, 'RowNames', arrayfun(@(r) sprintf('robot%d', r), 1:numTeam, 'UniformOutput', false)));

%% Diagnostics: where do the extra tracks come from?
labelsAll = labels;
printDiagnostics(copies, params, labelsAll, truth, viewRobot, debugLog, debugMaxLines);

%% Plots -- the viewRobot copy's tracks, grouped by TrueLabel for layout only

tracks         = copies(viewRobot).tracks;
targetNames    = {tracks.Name};
targetQueues   = {tracks.queue};
initialSystems = {tracks.initialSystem};
trueLabels     = {tracks.TrueLabel};
displayNames   = cellfun(@(n, tl, al) sprintf('%s (%s)%s', n, tl, repmat(' DELETED', 1, ~al)), ...
    targetNames, trueLabels, {tracks.alive}, 'UniformOutput', false);

mergedEvents = Event.empty;
for n = 1:numel(targetQueues)
    mergedEvents = [mergedEvents, targetQueues{n}]; %#ok<AGROW>
end
mergedEvents = sort(mergedEvents);

isOpp = startsWith(trueLabels, 'opp');

plotStatesGrid(targetQueues(isOpp), displayNames(isOpp), 1, sprintf('Opponents (robot %d copy)', viewRobot));
plotStatesGrid(targetQueues(~isOpp), displayNames(~isOpp), 2, sprintf('Ball + teammates (robot %d copy)', viewRobot));
plotSigma(targetQueues, displayNames, 3);
plotFieldTrajectory(targetQueues, displayNames, robots);
plotAliveTracks(packets, aliveHist, 5);
stepThroughMultiTargetTracking(mergedEvents, targetQueues, targetNames, initialSystems, robots);

%% Local functions

function y = simulateRangeBearing(pkt, truePos)
%SIMULATERANGEBEARING One noisy [unit bearing; range] measurement of
%   TRUEPOS, drawn from the same distribution MEASUREMENTBALLBEARING/
%   NOISEDENSITY assumes (isotropic bearing block with the tangential
%   variance, independent range with its fixed std plus the observer's
%   line-of-sight position variance), but evaluated at the TRUE geometry
%   -- what a real sensor sees -- rather than at a track's prior mean.

d = truePos - pkt.ObserverPosition;
r = norm(d);
u = d / r;
t = [-u(2); u(1)];

sigmaTangential2 = pkt.DetectionNoiseStd^2 + pkt.ObserverHeadingVariance ...
    + (t.' * pkt.ObserverCovariance * t) / r^2;
sigmaRange2 = pkt.RangeNoiseStd^2 + u.' * pkt.ObserverCovariance * u;

y = [u + sqrt(sigmaTangential2)*randn(2, 1); r + sqrt(sigmaRange2)*randn()];
end

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

dets = pkt.detections;
nd   = numel(dets);
nt   = numel(tracks);

% Squared Mahalanobis distance of every detection to every live track of
% its class. D2S uses the full innovation covariance S = H*P*H' + R (the
% right gate); D2R uses the measurement noise R alone (what the first
% version gated on -- logged only so the two can be compared).
D2S = inf(nd, nt);
D2R = inf(nd, nt);
for i = 1:nd
    mScore = makeMeasurement(pkt, dets(i));
    for k = find([tracks.alive] & strcmp({tracks.Class}, dets(i).Class))
        [D2S(i, k), D2R(i, k)] = gateStats(mScore, tracks(k).system, dets(i).y);
    end
end

% Assignment: ASSIGN(i) = track index detection i is fused into, 0 = spawn.
assign = zeros(1, nd);
C = D2S;
C(C > params.gateThreshold) = inf;
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

seen = false(1, nt);
took = zeros(1, nd); % index of the track each detection ended up in
for i = 1:nd
    d = dets(i);
    if assign(i) == 0
        [newTrack, ev] = spawnTrack(tracks, pkt, d, params, truth, D2S(i, :), D2R(i, :));
        % Fuse the detection into the vague prior right away (so the new
        % track has a first update and a history to plot), but do not log
        % it as an association -- it would pollute the d2 calibration.
        newTrack = fuseDetection(newTrack, pkt, d, 0, 0, truth);
        tracks(end + 1) = newTrack; %#ok<AGROW>
        k = numel(tracks);
    else
        k = assign(i);
        [tracks(k), ev] = fuseDetection(tracks(k), pkt, d, D2S(i, k), D2R(i, k), truth);
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
        if took(a) ~= took(b)
            tracks(took(a)).distinct = union(tracks(took(a)).distinct, took(b));
            tracks(took(b)).distinct = union(tracks(took(b)).distinct, took(a));
        end
    end
end

% Deletion: only tracks that SHOULD have been in this scan but weren't
% fused count a miss; a track merely out of view is left alone.
for k = find([tracks.alive] & ~seen)
    if expectedVisible(tracks(k).system.density.mean(), pkt, params)
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

if ~isnan(params.mergeThreshold)
    [tracks, mevs] = mergeDuplicates(tracks, t, params);
    evs = [evs, mevs];
end
end

function [d2S, d2R] = gateStats(m, sys, y)
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
d2R = nu.' * (R \ nu);
end

function [tracks, evs] = mergeDuplicates(tracks, t, params)
%MERGEDUPLICATES Drop duplicate live tracks: two same-class tracks whose
%   position estimates agree within the merge gate (position difference
%   against the SUM of their covariances) are the same object tracked
%   twice. The one with fewer fused detections is deleted (the younger on
%   a tie).

evs = makeEvent('x', 0, '', '', '', 0, 0, 0, false, '', 0);
evs(1) = [];

idx = find([tracks.alive]);
n   = numel(idx);
mu  = zeros(2, n);
P   = zeros(2, 2, n);
for a = 1:n
    m = tracks(idx(a)).system.density.mean();
    C = tracks(idx(a)).system.density.cov();
    mu(:, a)    = m(1:2);
    P(:, :, a) = C(1:2, 1:2);
end

for a = 1:n - 1
    for b = a + 1:n
        i = idx(a);
        j = idx(b);
        if ~tracks(i).alive || ~tracks(j).alive || ~strcmp(tracks(i).Class, tracks(j).Class) ...
                || ismember(j, tracks(i).distinct)
            continue
        end
        dm = mu(:, a) - mu(:, b);
        d2 = dm.' * ((P(:, :, a) + P(:, :, b)) \ dm);
        if d2 > params.mergeThreshold
            continue
        end
        ni = numel(tracks(i).queue);
        nj = numel(tracks(j).queue);
        if ni > nj || (ni == nj && tracks(i).SpawnTime <= tracks(j).SpawnTime)
            keep = i; drop = j;
        else
            keep = j; drop = i;
        end
        tracks(drop).alive        = false;
        tracks(drop).deletedAt    = t;
        tracks(drop).DeleteReason = 'merged';
        evs = [evs, makeEvent('merge', t, tracks(drop).Name, tracks(drop).TrueLabel, tracks(keep).TrueLabel, ...
            d2, 0, numel(tracks(drop).queue), strcmp(tracks(drop).TrueLabel, tracks(keep).TrueLabel), ...
            tracks(keep).Name, t - tracks(drop).SpawnTime)]; %#ok<AGROW>
    end
end
end

function tf = expectedVisible(mu, pkt, params)
%EXPECTEDVISIBLE Would a target at estimated position MU be comfortably
%   inside this scan's FOV wedge and range? "Comfortably" = within
%   PARAMS.EXPECTEDMARGIN of the half-angle and range (the detector's
%   confidence fades toward both edges, so edge misses are normal), and
%   not so close to the sender that it's the sender itself.

d = mu(1:2) - pkt.ObserverPosition;
r = norm(d);
angErr = atan2(sin(atan2(d(2), d(1)) - pkt.LookHeading), cos(atan2(d(2), d(1)) - pkt.LookHeading));

tf = r > params.selfExclusion ...
    && r <= params.expectedMargin * pkt.Range ...
    && abs(angErr) <= params.expectedMargin * pkt.FOV / 2;
end

function [track, ev] = fuseDetection(track, pkt, d, d2S, d2R, truth)
%FUSEDETECTION Fuse detection D into TRACK (already predicted to the
%   scan time) and return the debug event.

m = makeMeasurement(pkt, d);
m.AlreadyDetected = true;
m.LastDetection   = d.LastDetection;
m.TargetName      = track.Name;
m.needToSimulate  = false;

track.system.x_sim = trueStateAt(track.TrueLabel, pkt.time, truth); % ground truth, for plotting only
track.misses = 0;

% MEASUREMENTBALLBEARING/UPDATE already catches and warns on optimiser
% non-convergence itself (leaving the density at its predict-only state).
[m, track.system] = m.process(track.system);
track.queue(end + 1) = m;

ev = makeEvent('assoc', pkt.time, track.Name, track.TrueLabel, d.TrueLabel, d2S, d2R, ...
    numel(track.queue), strcmp(track.TrueLabel, d.TrueLabel), '', 0);
end

function m = makeMeasurement(pkt, d)
%MAKEMEASUREMENT The MEASUREMENTBALLBEARING for detection D of packet PKT.
m = MeasurementBallBearing();
m.time                    = pkt.time;
m.ObserverPosition        = pkt.ObserverPosition;
m.ObserverCovariance      = pkt.ObserverCovariance;
m.DetectionNoiseStd       = pkt.DetectionNoiseStd;
m.RangeNoiseStd           = pkt.RangeNoiseStd;
m.ObserverHeadingVariance = pkt.ObserverHeadingVariance;
m.RobotID                 = pkt.RobotID;
m.y                       = d.y;
m.verbosity               = 0;
m.updateMethod            = 'BFGSTrustSqrt';
end

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
mu0  = [pkt.ObserverPosition + r*u; 0; 0];

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

function ev = makeEvent(type, t, name, trackLabel, detLabel, d2S, d2R, n, flag, note, life)
%MAKEEVENT One row of the debug log. Fields are reused by type:
%   assoc : detection fused into track NAME. trackLabel/detLabel are the
%           REAL objects (ground truth, for diagnosis only); flag = they
%           match; d2S/d2R = Mahalanobis distance (full S / R only); n =
%           fused detections the track has now.
%   spawn : new track NAME. d2S/d2R = distance to the closest still-live
%           track of the same real object (inf if none), n = how many
%           such live tracks, note = first | respawn | duplicate.
%   delete: track NAME removed after maxMisses expected-but-unseen scans;
%           n = detections it had fused, life = seconds it lived, flag =
%           it was the last live track for that real object.
%   merge : track NAME dropped as a duplicate of the track named in NOTE
%           (detLabel = that track's real object), d2S = merge distance,
%           flag = same real object (a correct merge).
ev = struct('type', type, 't', t, 'name', name, 'trackLabel', trackLabel, 'detLabel', detLabel, ...
    'd2S', d2S, 'd2R', d2R, 'n', n, 'flag', flag, 'note', note, 'life', life);
end

function printDiagnostics(copies, params, labels, truth, viewRobot, debugLog, maxLines)
%PRINTDIAGNOSTICS Terminal summary of where tracks come from and go,
%   built from each copy's event log (see MAKEEVENT).

qs = @(x) fmtQ(x(isfinite(x)));

fprintf(1, '\n================ Diagnostics ================\n');
fprintf(1, 'Association gate: d2 <= %.2f (full innovation covariance, 3 dof; oneToOne=%d). A correctly associated\n', ...
    params.gateThreshold, params.oneToOne);
fprintf(1, 'detection should have d2 ~ chi2(3): median 2.37, 90%% 6.25, 99%% 11.34.\n');
fprintf(1, 'Merge gate: d2 <= %.2f (2 dof position difference).\n\n', params.mergeThreshold);

fprintf(1, 'Per copy:\n');
fprintf(1, ' robot | fusions  wrongLabel | spawns: first respawn duplicate | removed: missed merged(wrong) | short(<=2 fused) lastForObject\n');
nC = numel(copies);
spawnsPerLabel = zeros(nC, numel(labels));
for r = 1:nC
    L = copies(r).log;
    types = {L.type};
    A = L(strcmp(types, 'assoc'));
    S = L(strcmp(types, 'spawn'));
    D = L(strcmp(types, 'delete'));
    M = L(strcmp(types, 'merge'));
    notes = {S.note};
    fprintf(1, '  %d    | %6d  %6d     |       %5d %7d %9d     |         %6d %6d (%d)       | %9d %14d\n', r, ...
        numel(A), sum(~[A.flag]), ...
        sum(strcmp(notes, 'first')), sum(strcmp(notes, 'respawn')), sum(strcmp(notes, 'duplicate')), ...
        numel(D), numel(M), sum(~[M.flag]), ...
        sum([D.n] <= 2) + sum([M.n] <= 2), sum([D.flag]));
    for e = 1:numel(S)
        spawnsPerLabel(r, strcmp(labels, S(e).trackLabel)) = spawnsPerLabel(r, strcmp(labels, S(e).trackLabel)) + 1;
    end
end

allL = [copies.log];
types = {allL.type};
A = allL(strcmp(types, 'assoc'));
S = allL(strcmp(types, 'spawn'));
D = allL(strcmp(types, 'delete'));
M = allL(strcmp(types, 'merge'));

okA  = A([A.flag]);
badA = A(~[A.flag]);
fprintf(1, '\nAll copies, fused detections whose track IS that real object (%d): d2 percentiles [50 90 99]\n', numel(okA));
fprintf(1, '   full S : %s     (R only: %s)\n', qs([okA.d2S]), qs([okA.d2R]));
if ~isempty(badA)
    fprintf(1, 'Fused into a track of a DIFFERENT real object (%d): d2S percentiles %s\n', numel(badA), qs([badA.d2S]));
    pairs = strcat({badA.detLabel}, '->', {badA.trackLabel});
    [u, ~, ic] = unique(pairs);
    cnt = accumarray(ic(:), 1);
    [~, ord] = sort(cnt, 'descend');
    fprintf(1, '   most common (detection->track): ');
    for o = ord(1:min(5, numel(ord))).'
        fprintf(1, '%s x%d  ', u{o}, cnt(o));
    end
    fprintf(1, '\n');
end

notes = {S.note};
dup = S(strcmp(notes, 'duplicate'));
fprintf(1, '\nDuplicate spawns (%d): distance to the closest live track of the SAME real object\n', numel(dup));
if ~isempty(dup)
    fprintf(1, '   full S : %s   -> %d inside the gate (lost the one-to-one assignment), %d outside it\n', ...
        qs([dup.d2S]), sum([dup.d2S] <= params.gateThreshold), sum([dup.d2S] > params.gateThreshold));
    fprintf(1, '   R only : %s   (what the old gate used)\n', qs([dup.d2R]));
end
fprintf(1, 'Spawns by reason: first=%d respawn=%d duplicate=%d\n', ...
    sum(strcmp(notes, 'first')), sum(strcmp(notes, 'respawn')), numel(dup));

if ~isempty(D)
    fprintf(1, '\nDeleted for being expected-but-unseen (%d): lifetime [s] percentiles %s; fused detections %s\n', ...
        numel(D), qs([D.life]), qs([D.n]));
    fprintf(1, '   %d of them were the LAST live track for their real object (a genuine loss, not a duplicate clearing).\n', sum([D.flag]));
end
if ~isempty(M)
    fprintf(1, 'Merged duplicates (%d): %d joined tracks of the same real object, %d of DIFFERENT objects (bad merges)\n', ...
        numel(M), sum([M.flag]), sum(~[M.flag]));
end

fprintf(1, '\nSpawns per real object (rows = robot copies, cols = real objects; ideal is 1):\n');
disp(array2table(spawnsPerLabel, 'VariableNames', labels, ...
    'RowNames', arrayfun(@(r) sprintf('robot%d', r), 1:nC, 'UniformOutput', false)));

tr = copies(viewRobot).tracks;
fprintf(1, 'Live tracks in robot %d''s copy:\n', viewRobot);
fprintf(1, '  name    real object  class  spawned  fused  misses  pos err [m]\n');
for k = find([tr.alive])
    mu = tr(k).system.density.mean();
    xTrue = trueStateAt(tr(k).TrueLabel, tr(k).system.time, truth);
    fprintf(1, '  %-7s %-11s  %-5s  %6.2fs  %5d  %6d  %8.3f\n', tr(k).Name, tr(k).TrueLabel, tr(k).Class, ...
        tr(k).SpawnTime, numel(tr(k).queue), tr(k).misses, norm(mu(1:2) - xTrue(1:2)));
end

if debugLog
    L = copies(viewRobot).log;
    types = {L.type};
    keep = strcmp(types, 'spawn') | strcmp(types, 'delete') | strcmp(types, 'merge') | (strcmp(types, 'assoc') & ~[L.flag]);
    L = L(keep);
    fprintf(1, '\nEvent log, robot %d copy (spawn / delete / merge / wrong-object fusion), first %d of %d:\n', ...
        viewRobot, min(maxLines, numel(L)), numel(L));
    for e = 1:min(maxLines, numel(L))
        ev = L(e);
        switch ev.type
            case 'spawn'
                fprintf(1, '  [t=%6.2f] SPAWN  %-6s real=%-6s %-9s liveSameObject=%d closest d2S=%.1f (R only %.1f)\n', ...
                    ev.t, ev.name, ev.trackLabel, ev.note, ev.n, ev.d2S, ev.d2R);
            case 'delete'
                fprintf(1, '  [t=%6.2f] DELETE %-6s real=%-6s fused=%d lived=%.1fs lastTrackForObject=%d\n', ...
                    ev.t, ev.name, ev.trackLabel, ev.n, ev.life, ev.flag);
            case 'merge'
                fprintf(1, '  [t=%6.2f] MERGE  %-6s real=%-6s into %s (real=%s) d2=%.2f fused=%d sameObject=%d\n', ...
                    ev.t, ev.name, ev.trackLabel, ev.note, ev.detLabel, ev.d2S, ev.n, ev.flag);
            case 'assoc'
                fprintf(1, '  [t=%6.2f] WRONG  detection of %-6s fused into %-6s (real=%s) d2S=%.1f\n', ...
                    ev.t, ev.detLabel, ev.name, ev.trackLabel, ev.d2S);
        end
    end
end
fprintf(1, '=============================================\n\n');
end

function s = fmtQ(x)
%FMTQ "[p50 p90 p99]" of X, or "n/a" if empty.
if isempty(x)
    s = 'n/a';
else
    p = prctile(x, [50 90 99]);
    s = sprintf('[%.2f %.2f %.2f]', p(1), p(2), p(3));
end
end

function x = trueStateAt(label, t, truth)
%TRUESTATEAT Ground-truth [position; velocity] for LABEL at time T,
%   looked up from the dense grids precomputed above.

kk = min(max(round(t/truth.dtSim) + 1, 1), truth.Nt);

if strcmp(label, 'ball')
    x = [truth.ballPos(:, kk); truth.ballVel(:, kk)];
elseif startsWith(label, 'opp')
    o = sscanf(label, 'opp%d');
    x = [truth.oppPos{o}(:, kk); truth.oppVel{o}(:, kk)];
elseif startsWith(label, 'team')
    j = sscanf(label, 'team%d');
    x = [truth.robots(j).Position(:); 0; 0];
else
    error('Unknown TrueLabel ''%s''', label);
end
end

function plotAliveTracks(packets, aliveHist, fig)
%PLOTALIVETRACKS Live-track count in every robot's copy over time --
%   copies diverge when links drop packets, and counts fall when tracks
%   are deleted.

hf = figure(fig); clf(fig);
hf.Position = [450, 450, 700, 400];
ax = axes(hf);
hold(ax, 'on')
tt = [packets.time];
for r = 1:size(aliveHist, 1)
    stairs(ax, tt, aliveHist(r, :), 'DisplayName', sprintf('robot %d', r))
end
hold(ax, 'off')
grid(ax, 'on')
xlabel(ax, 'Time [s]')
ylabel(ax, 'Live tracks')
title(ax, 'Live tracks in each robot''s own copy')
legend(ax, 'Location', 'best')
end

function h = plotStatesGrid(targetQueues, targetNames, fig, figTitle)
%PLOTSTATESGRID 4 (state) x N (target) grid: one column per tracked
%   thing, each cell true/estimate/+-3sigma for that state.

if nargin < 4
    figTitle = '';
end

numTargets = numel(targetQueues);
stateNames = {'x position [m]', 'y position [m]', 'x velocity [m/s]', 'y velocity [m/s]'};
n_sigma = 3;

hf = figure(fig); clf(fig);
hf.Position = [50, 50, max(230*numTargets, 400), 760];

ax = gobjects(4, max(numTargets, 1));
for n = 1:numTargets
    q = targetQueues{n};
    N = length(q);
    t_hist     = nan(1, N);
    x_hist     = nan(4, N);
    mu_hist    = nan(4, N);
    sigma_hist = nan(4, N);
    for k = 1:N
        t_hist(:, k) = q(k).time;
        if q(k).saveSystemState
            x_hist(:, k)     = q(k).system.x_sim;
            mu_hist(:, k)    = q(k).system.density.mean();
            sigma_hist(:, k) = realsqrt(diag(q(k).system.density.cov()));
        end
    end
    mu_hist_m = mu_hist - n_sigma*sigma_hist;
    mu_hist_p = mu_hist + n_sigma*sigma_hist;

    for i = 1:4
        ax(i, n) = subplot(4, numTargets, (i - 1)*numTargets + n, 'Parent', hf);
        hold(ax(i, n), 'on')
        plot(ax(i, n), t_hist, x_hist(i, :), 'r')
        plot(ax(i, n), t_hist, mu_hist(i, :), 'Color', [0.1 0.7 0.2])
        fill(ax(i, n), [t_hist, fliplr(t_hist)], [mu_hist_m(i, :), fliplr(mu_hist_p(i, :))], [0.1 0.7 0.2], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        hold(ax(i, n), 'off')
        grid(ax(i, n), 'on')
        if n == 1
            ylabel(ax(i, n), stateNames{i})
        end
        if i == 1
            title(ax(i, n), targetNames{n})
        end
        if i == 4
            xlabel(ax(i, n), 'Time [s]')
        end
    end
end
if numTargets > 0
    legend(ax(1, 1), {'true', 'estimate', '\pm3\sigma'}, 'Location', 'best')
    ax(1, 1).XLimitMethod = 'tight';
    h.link = linkprop(ax(:, 1:numTargets), {'XLim'});
end
if ~isempty(figTitle)
    sgtitle(hf, figTitle)
end

h.hf = hf;
h.ax = ax;
end

function h = plotSigma(targetQueues, targetNames, fig)
%PLOTSIGMA Marginal standard deviations for all targets overlaid.

numTargets = numel(targetQueues);
targetColors = lines(max(numTargets, 1));
stateNames = {'x position [m]', 'y position [m]', 'x velocity [m/s]', 'y velocity [m/s]'};

hf = figure(fig); clf(fig);
hf.Position = [400, 400, 900, 700];

ax = gobjects(1, 4);
for i = 1:4
    ax(i) = subplot(2, 2, i, 'Parent', hf);
    hold(ax(i), 'on')
end

legendHandles = gobjects(1, numTargets);
for n = 1:numTargets
    q = targetQueues{n};
    N = length(q);
    t_hist     = nan(1, N);
    sigma_hist = nan(4, N);
    for k = 1:N
        t_hist(:, k) = q(k).time;
        if q(k).saveSystemState
            sigma_hist(:, k) = realsqrt(diag(q(k).system.density.cov()));
        end
    end
    c = targetColors(n, :);
    for i = 1:4
        hLine = semilogy(ax(i), t_hist, sigma_hist(i, :), 'Color', c);
        if i == 1
            legendHandles(n) = hLine;
        end
    end
end

for i = 1:4
    hold(ax(i), 'off')
    xlabel(ax(i), 'Time [s]')
    ylabel(ax(i), ['\sigma, ' stateNames{i}])
    grid(ax(i), 'on')
end
if numTargets > 0
    legend(ax(1), legendHandles, targetNames, 'Location', 'best')
end
title(ax(1), 'Marginal standard deviations (all tracks)', 'FontSize', 14)

h.hf = hf;
h.ax = ax;
ax(1).XLimitMethod = 'tight';
h.link = linkprop(ax, {'XLim'});
end

function plotFieldTrajectory(targetQueues, targetNames, robots)
%PLOTFIELDTRAJECTORY Top-down view of every track's true and estimated
%   path, plus each stationary sensing robot's FOV cone.

ax = drawSoccerField(fieldConfig());

for r = 1:numel(robots)
    player.Position   = robots(r).Position.';
    player.Heading     = robots(r).Heading;
    player.Covariance = robots(r).Covariance;
    player.FOV         = robots(r).FOV;
    player.Range       = robots(r).Range;
    drawPlayer(ax, player);
    text(ax, robots(r).Position(1), robots(r).Position(2) - 0.6, sprintf('R%d', r), ...
        'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
end

targetColors = lines(max(numel(targetQueues), 1));
legendHandles = gobjects(1, 2*numel(targetQueues));
legendLabels  = cell(1, 2*numel(targetQueues));

for n = 1:numel(targetQueues)
    q = targetQueues{n};
    N = length(q);
    x_hist  = nan(4, N);
    mu_hist = nan(4, N);
    for k = 1:N
        if q(k).saveSystemState
            x_hist(:, k)  = q(k).system.x_sim;
            mu_hist(:, k) = q(k).system.density.mean();
        end
    end
    c = targetColors(n, :);
    legendHandles(2*n - 1) = plot(ax, x_hist(1, :), x_hist(2, :), '-', 'Color', c, 'LineWidth', 1.5);
    legendLabels{2*n - 1}  = sprintf('%s true', targetNames{n});
    legendHandles(2*n)     = plot(ax, mu_hist(1, :), mu_hist(2, :), '--', 'Color', c, 'LineWidth', 1.5);
    legendLabels{2*n}      = sprintf('%s estimate', targetNames{n});
end

legend(ax, legendHandles, legendLabels, 'TextColor', 'w', 'Location', 'best');
title(ax, 'True vs. estimated paths -- data-association tracks', 'Color', 'w');
end
