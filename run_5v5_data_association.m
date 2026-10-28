% RUN_5V5_DATA_ASSOCIATION 

clc;
clear all;

verbosity = 1;  % 0: silent, 1: dots, 2: summary, 3: iter

% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Stationary sensing robots -- same 5v5 formation as
% RUN_5V5_STATIONARY_ROBOTS (our five on x<0 facing forward, head-panning
% +-80 degrees)
clear robots

RANGE_SCALE = 1.8;

%create our team robots stationary for now
robots(1).Position          = [-8; -4];
robots(1).Covariance        = diag([0.02, 0.02]);
robots(1).DetectionNoiseStd = deg2rad(0.75);
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
robots(5).Rate               = 3;
robots(5).Heading           = 0;
robots(5).FOV               = deg2rad(120);
robots(5).Range             = RANGE_SCALE * 12;
robots(5).GazeAmplitude      = deg2rad(80);
robots(5).GazeAngularSpeed   = 2*pi/6;
robots(5).GazePhase0         = 8*pi/5;

T_total = 15; % seconds

%% Ground truth for everything moving things %%disclaimer AI gave me this I
% just validated ground truth movements visually

cfg   = fieldConfig();
dtSim = 0.02;
tGrid = 0:dtSim:T_total;
Nt    = numel(tGrid);

% Ball: same scripted kicks
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

% should be a friction model that matches our process
% model
for kk = 1:Nt
    ballPos(:, kk) = bx;
    ballVel(:, kk) = bv;
    if kk < Nt
        bx = bx + bv*dtSim;
        tNext = tGrid(kk + 1);
        while nextKick <= size(kickVelocities, 1) && tNext >= kickTimes(nextKick)
            bv = bv + kickVelocities(nextKick, :).';
            nextKick = nextKick + 1;
        end
    end
end

% Opposition robots, same 5-robot smooth random walk as RUN_5V5_STATIONARY_ROBOTS.
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

%% Build the detection queue
% One robot's scan at one instant can see several real objects
% at once -- the ball, teammates, opponents -- as what yolo would produce. 
% 

numTeam = numel(robots);


detections = struct('time', {}, 'RobotID', {}, 'ObserverPosition', {}, ...
    'ObserverCovariance', {}, 'DetectionNoiseStd', {}, 'ObserverHeadingVariance', {}, ...
    'y', {}, 'TrueLabel', {}, 'LastDetection', {});

for r = 1:numTeam
    for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
        kk = min(max(round(t/dtSim) + 1, 1), Nt);

        candidates = struct('Label', {}, 'Position', {});
        candidates(end + 1) = struct('Label', 'ball', 'Position', ballPos(:, kk)); %#ok<SAGROW>
        for o = 1:numOpp
            candidates(end + 1) = struct('Label', sprintf('opp%d', o), 'Position', oppPos{o}(:, kk)); %#ok<SAGROW>
        end
        for j = 1:numTeam
            if j == r
                continue % a robot doesn't detect itself
            end
            candidates(end + 1) = struct('Label', sprintf('team%d', j), 'Position', robots(j).Position); %#ok<SAGROW>
        end

        gazeOffset = robots(r).GazeAmplitude * sin(robots(r).GazePhase0 + robots(r).GazeAngularSpeed*t);
        sensor.Position   = robots(r).Position.';
        sensor.Heading    = robots(r).Heading;
        sensor.GazeOffset = gazeOffset;
        sensor.FOV        = robots(r).FOV;
        sensor.Range      = robots(r).Range;

        for c = 1:numel(candidates)
            obj_.Position = candidates(c).Position.';
            det = computeBallDetection(sensor, obj_);
            if ~det.Detected
                continue
            end
            y = simulateBearingDetection(robots(r).Position, robots(r).Covariance, ...
                robots(r).DetectionNoiseStd, 0, candidates(c).Position);
            detections(end + 1) = struct('time', t, 'RobotID', r, ...
                'ObserverPosition', robots(r).Position, 'ObserverCovariance', robots(r).Covariance, ...
                'DetectionNoiseStd', robots(r).DetectionNoiseStd, 'ObserverHeadingVariance', 0, ...
                'y', y, 'TrueLabel', candidates(c).Label, 'LastDetection', det); %#ok<SAGROW>
        end
    end
end

rng(s);

[~, order] = sort([detections.time]);
detections = detections(order);

fprintf(1, 'Generated %d raw detections across %d sensing robots.\n', numel(detections), numTeam);

%% Online data association + tracking
% See header comment for the algorithm. TRACKS grows dynamically -- one
% entry per spawned track, in spawn order.

gateSigma = 3; % same 3-sigma confidence-region convention used throughout this repo (e.g. PLOTSTATESGRID's n_sigma)

% Ground truth, bundled just so it can be threaded through as one
% argument for TRUESTATEAT's plotting-only lookups (see header comment).
truth.dtSim  = dtSim;
truth.Nt     = Nt;
truth.ballPos = ballPos;
truth.ballVel = ballVel;
truth.oppPos  = oppPos;
truth.oppVel  = oppVel;
truth.robots  = robots;

tracks = struct('Name', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {});

for di = 1:numel(detections)
    d = detections(di);
    tracks = associateAndUpdate(tracks, d, gateSigma, verbosity, truth);
    if verbosity > 0 && mod(di, 200) == 0
        fprintf(1, '  [%d/%d detections processed, %d tracks live]\n', di, numel(detections), numel(tracks));
    end
end

fprintf(1, 'Finished with %d tracks from %d real objects on the field.\n', numel(tracks), 1 + numOpp + numTeam);

%% Plots
% Grouped by TrueLabel purely for figure layout (see header comment --
% this bookkeeping is never used by the associator itself): opponents in
% one states-grid window, ball + our own robots (teammates) in another,
% same split RUN_5V5_STATIONARY_ROBOTS used. Track names show both the
% associator's own anonymous ID and (for us to sanity-check association)
% which real object it actually locked onto.

targetNames    = {tracks.Name};
targetQueues   = {tracks.queue};
initialSystems = {tracks.initialSystem};
trueLabels     = {tracks.TrueLabel};
displayNames   = cellfun(@(n, tl) sprintf('%s (%s)', n, tl), targetNames, trueLabels, 'UniformOutput', false);

mergedEvents = Event.empty;
for n = 1:numel(targetQueues)
    mergedEvents = [mergedEvents, targetQueues{n}]; %#ok<AGROW>
end
mergedEvents = sort(mergedEvents);

isOpp = startsWith(trueLabels, 'opp');

plotStatesGrid(targetQueues(isOpp), displayNames(isOpp), 1, 'Opponents (data association)');
plotStatesGrid(targetQueues(~isOpp), displayNames(~isOpp), 2, 'Ball + teammates (data association)');
plotSigma(targetQueues, displayNames, 3);
plotFieldTrajectory(targetQueues, displayNames, robots);
stepThroughMultiTargetTracking(mergedEvents, targetQueues, targetNames, initialSystems, robots);

%% Local functions

function y = simulateBearingDetection(observerPos, observerCov, detNoiseStd, headingVar, truePos)
%SIMULATEBEARINGDETECTION Draw one noisy unit-bearing measurement from
%   TRUEPOS, using exactly the same tangential/depth noise decomposition
%   as MEASUREMENTBALLBEARING/NOISEDENSITY -- just evaluated at the true
%   geometry (what a real sensor actually sees) rather than at a track's
%   prior mean (what NOISEDENSITY uses for fusion linearisation).

d = truePos - observerPos;
r = norm(d);
u = d / r;
t = [-u(2); u(1)];

sigmaTangential2 = detNoiseStd^2 + headingVar + (t.' * observerCov * t) / r^2;
assumedDepthStd = 1; % m -- matches MEASUREMENTBALLBEARING/NOISEDENSITY's assumed depth precision

y = u + t*sqrt(sigmaTangential2)*randn() + u*assumedDepthStd*randn();
end

function tracks = associateAndUpdate(tracks, d, gateSigma, verbosity, truth)
%ASSOCIATEANDUPDATE Nearest-neighbour gated data association for one raw
%   detection D against the current TRACKS, then fuse it into whichever
%   track wins (spawning a new one if nothing gates). TRUTH is only used
%   to stamp the matched track's x_sim for plotting (see header comment).

bestIdx = 0;
bestD2  = inf;

mScore = MeasurementBallBearing();
mScore.ObserverPosition        = d.ObserverPosition;
mScore.ObserverCovariance      = d.ObserverCovariance;
mScore.DetectionNoiseStd       = d.DetectionNoiseStd;
mScore.ObserverHeadingVariance = d.ObserverHeadingVariance;

for k = 1:numel(tracks)
    predSys = tracks(k).system.predict(d.time);
    py = mScore.predictDensity(predSys.density.mean(), predSys);
    if py.isWithinConfidenceRegion(d.y, gateSigma)
        w = py.Xi*d.y - py.nu;
        d2 = w.'*w;
        if d2 < bestD2
            bestD2 = d2;
            bestIdx = k;
        end
    end
end

if bestIdx == 0
    % Nothing gated -- spawn a new track, back-projecting the bearing to
    % the same ~1m assumed depth NOISEDENSITY uses, with a loose prior
    % (position uncertainty from that depth assumption, velocity totally
    % unknown at spawn).
    assumedDepth = 1; % m
    posGuess = d.ObserverPosition + assumedDepth*d.y;
    mu0 = [posGuess; 0; 0];
    S0  = diag([1.5, 1.5, 2.5, 2.5]);

    newTrack.Name           = sprintf('trk%d', numel(tracks) + 1);
    newTrack.TrueLabel      = d.TrueLabel;
    newTrack.system         = SystemBall();
    newTrack.system.time    = d.time;
    newTrack.system.density = GaussianInfo.fromSqrtMoment(mu0, S0);
    newTrack.system.x_sim   = trueStateAt(d.TrueLabel, d.time, truth); % ground truth, for INITIALSYSTEMS's pre-first-event display only
    newTrack.initialSystem  = newTrack.system;
    newTrack.queue          = MeasurementBallBearing.empty;

    tracks(end + 1) = newTrack;
    bestIdx = numel(tracks);
end

m = MeasurementBallBearing();
m.time                    = d.time;
m.ObserverPosition        = d.ObserverPosition;
m.ObserverCovariance      = d.ObserverCovariance;
m.DetectionNoiseStd       = d.DetectionNoiseStd;
m.ObserverHeadingVariance = d.ObserverHeadingVariance;
m.RobotID                 = d.RobotID;
m.TargetName              = tracks(bestIdx).Name;
m.AlreadyDetected          = true;
m.LastDetection           = d.LastDetection;
m.y                       = d.y;
m.needToSimulate          = false;
m.verbosity               = verbosity;
m.updateMethod            = 'BFGSTrustSqrt';

% Ground-truth state, for plotting only (see header comment) -- not used
% by PROCESS/UPDATE at all since ALREADYDETECTED bypasses the one place
% (COMPUTEBALLDETECTION's re-check) that would otherwise read x_sim.
tracks(bestIdx).system.x_sim = trueStateAt(tracks(bestIdx).TrueLabel, d.time, truth);

% MEASUREMENTBALLBEARING/UPDATE already catches and warns on optimiser
% non-convergence itself (leaving density at its predict-only state) --
% no need to duplicate that handling here.
[m, tracks(bestIdx).system] = m.process(tracks(bestIdx).system);

tracks(bestIdx).queue(end + 1) = m;
end

function x = trueStateAt(label, t, truth)
%TRUESTATEAT Ground-truth [position; velocity] for LABEL at time T,
%   looked up from the dense grids this script precomputed -- for
%   plotting only (see header comment).

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
