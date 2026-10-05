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
gateSigma      = 3;     % confidence-region gate, same 3-sigma convention as elsewhere
maxMisses      = 5;     % consecutive expected-but-unseen scans before a track is deleted
expectedMargin = 0.8;   % fraction of FOV half-angle / range inside which a track counts as "expected" in a scan (the detector's confidence fades toward the edge)
selfExclusion  = 0.5;   % m, tracks this close to the sender are the sender itself, never expected
kappaBall      = 0.2;   % 1/s, ball rolling-friction velocity decay rate
ballAccelStd   = 0.5;   % m/s^2 per sqrt(Hz), fixed process noise for the ball (absorbs kicks)
robotAccelStd  = 0.1;   % same, for robots
T_total        = 15;    % seconds
viewRobot      = 1;     % whose copy the figures and step-through tool show

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

params.gateSigma      = gateSigma;
params.maxMisses      = maxMisses;
params.expectedMargin = expectedMargin;
params.selfExclusion  = selfExclusion;
params.kappaBall      = kappaBall;
params.ballAccelStd   = ballAccelStd;
params.robotAccelStd  = robotAccelStd;
params.verbosity      = verbosity;

aliveHist = zeros(numTeam, numPackets); % live tracks in each copy after each packet (carried forward when that copy missed it)
copies = struct('tracks', cell(1, numTeam));

for r = 1:numTeam
    tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, ...
        'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {});
    numAlive = 0;
    for p = 1:numPackets
        if delivered(r, p)
            tracks = processPacket(tracks, packets(p), params, truth);
            numAlive = sum([tracks.alive]);
        end
        aliveHist(r, p) = numAlive;
    end
    copies(r).tracks = tracks;
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

function tracks = processPacket(tracks, pkt, params, truth)
%PROCESSPACKET One robot's copy of the tracker handling one received
%   packet: predict, associate/fuse each detection, then do the
%   expected-but-unseen bookkeeping.

t = pkt.time;

% Bring every live track to the scan time once, up front (SYSTEM.PREDICT
% is a no-op for dt == 0, so the fusion step below doesn't redo it).
for k = find([tracks.alive])
    tracks(k).system = tracks(k).system.predict(t);
end

seen = false(1, numel(tracks));
for di = 1:numel(pkt.detections)
    [tracks, k] = associateAndUpdate(tracks, pkt, pkt.detections(di), params, truth);
    seen(k) = true; % a spawn grows TRACKS, so SEEN may need to grow too
end
seen(end + 1:numel(tracks)) = false;

% Deletion: only tracks that SHOULD have been in this scan but weren't
% fused count a miss; a track merely out of view is left alone.
for k = find([tracks.alive] & ~seen)
    if expectedVisible(tracks(k).system.density.mean(), pkt, params)
        tracks(k).misses = tracks(k).misses + 1;
        if tracks(k).misses >= params.maxMisses
            tracks(k).alive     = false;
            tracks(k).deletedAt = t;
        end
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

function [tracks, bestIdx] = associateAndUpdate(tracks, pkt, d, params, truth)
%ASSOCIATEANDUPDATE Nearest-neighbour gated data association for one
%   detection D from packet PKT against the live tracks of the same
%   class, then fuse it into the winner (spawning a new track if nothing
%   gates). Returns the index of the track that took it.

bestIdx = 0;
bestD2  = inf;

mScore = makeMeasurement(pkt, d);

for k = find([tracks.alive] & strcmp({tracks.Class}, d.Class))
    py = mScore.predictDensity(tracks(k).system.density.mean(), tracks(k).system);
    if py.isWithinConfidenceRegion(d.y, params.gateSigma)
        w = py.Xi*d.y - py.nu;
        d2 = w.'*w;
        if d2 < bestD2
            bestD2 = d2;
            bestIdx = k;
        end
    end
end

if bestIdx == 0
    tracks(end + 1) = spawnTrack(tracks, pkt, d, params, truth);
    bestIdx = numel(tracks);
end

m = mScore;
m.AlreadyDetected = true;
m.LastDetection   = d.LastDetection;
m.TargetName      = tracks(bestIdx).Name;
m.y               = d.y;
m.needToSimulate  = false;

tracks(bestIdx).system.x_sim = trueStateAt(tracks(bestIdx).TrueLabel, pkt.time, truth); % ground truth, for plotting only
tracks(bestIdx).misses = 0;

% MEASUREMENTBALLBEARING/UPDATE already catches and warns on optimiser
% non-convergence itself (leaving the density at its predict-only state).
[m, tracks(bestIdx).system] = m.process(tracks(bestIdx).system);
tracks(bestIdx).queue(end + 1) = m;
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

function track = spawnTrack(tracks, pkt, d, params, truth)
%SPAWNTRACK New track from one unassociated range-bearing detection:
%   the target is placed at the measured range along the measured
%   bearing, with the position covariance that measurement implies
%   (tangential r*sigma_t, radial sigma_r), velocity unknown. The
%   process model comes from the detector's class.

u = d.y(1:2) / norm(d.y(1:2));
t = [-u(2); u(1)];
r = d.y(3);

sigmaTangential2 = pkt.DetectionNoiseStd^2 + pkt.ObserverHeadingVariance ...
    + (t.' * pkt.ObserverCovariance * t) / r^2;
sigmaRange2 = pkt.RangeNoiseStd^2 + u.' * pkt.ObserverCovariance * u;

Ppos = r^2 * sigmaTangential2 * (t*t.') + sigmaRange2 * (u*u.');
P0   = blkdiag((Ppos + Ppos.')/2, 2.5^2*eye(2));
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
