% RUN_5V5_STATIONARY_ROBOTS 5v5: five of our own
%   stationary sensing robots track the ball, all five opposition
%   robots, AND each other -- every tracked thing (ball, 5 opponents, 5
%   teammates) is its own independent SYSTEMBALL instance/filter, not
%   one joint state, since nothing here couples their dynamics or their
%   measurements; a joint state would just carry a lot of always-zero
%   cross-covariance blocks for no benefit. Each event only ever updates
%   its own target's system; MEASUREMENTBALLBEARING.TargetName says
%   which. Same MEASUREMENTBALLBEARING detector throughout (same FOV/
%   confidence model, just pointed at a different target each time), and
%   the same "which robot did this measurement come from" visibility in
%   the step-through tool.
%
%   Teammates are tracked exactly like the ball/opponents are: robot i
%   reports a bearing to robot j (i~=j) using the SAME detector, and
%   that measurement updates a SYSTEMBALL instance representing j's
%   position, even though j's TRUE position is already known exactly
%   (it's one of our own fixed ROBOTS entries) -- this is deliberately
%   redundant here since none of our robots have any localisation
%   uncertainty of their own yet, but it's the same mechanism that would
%   matter once they do (cooperative localisation: a robot's teammates
%   confirming/refining its position estimate from their own bearings to
%   it).
%
%   Target identity is assumed perfectly known here (TargetName is set
%   directly when each measurement is built) -- with several opponents
%   (and now several teammates too) that could plausibly look alike to a
%   real detector, a real system would need an actual data-association
%   step to decide which target a given detection belongs to. Not
%   modelled in this script; noted as the obvious next thing to add.
%
%   The ball still moves via the same scripted kicks as
%   RUN_TRACK_BALL_STATIONARY_ROBOTS. Opponents do a smooth continuous
%   random walk (CREATEPLAYER/STEPPLAYERWALK, same model
%   RUN_MOVING_ROBOTS_STATIONARY_BALL used for robots) -- since that's a
%   continuously-curving true path rather than discrete kicks, each
%   opponent's SYSTEMBALL.x_sim is overwritten with its true position/
%   velocity at every event (see the loop below), the continuous-path
%   analogue of how the ball's kicks override x_sim. Teammates need no
%   such override at all: they're exactly stationary, so SYSTEMBALL's
%   own (zero-velocity) dynamics already keep x_sim exactly put.
%
%   This is a noticeably heavier run than the smaller scripts: 5 sensing
%   robots x (1 ball + 5 opponents + 4 other teammates each) = 50
%   (robot, target) pairs, each firing measurements at that robot's own
%   rate over T_total -- several thousand events, many triggering a
%   BFGSTrustSqrt solve. Expect it to take a while.

clc;
clear all;

verbosity = 1;  % 0: silent, 1: dots, 2: summary, 3: iter

% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Stationary sensing robots -- all five start on OUR half (x<0), facing
% forward (+x, towards the opposition's half). Heading is each robot's
% fixed BODY-forward direction; GazeAmplitude/GazeAngularSpeed/GazePhase0
% is an independent head-pan sweep on top of that heading -- same
% GazeOffset concept CREATEPLAYER/STEPPLAYERWALK already use for the 2D
% single-player demo's look direction, computed here in closed form each
% measurement's time (t) since these robots don't walk -- see
% BUILDMEASUREMENT below.
clear robots

robots(1).Position          = [-8; -4];
robots(1).Covariance        = diag([0.02, 0.02]);
robots(1).DetectionNoiseStd = deg2rad(0.75);
robots(1).Rate               = 2;
robots(1).Heading           = 0;
robots(1).FOV               = deg2rad(100);
robots(1).Range             = 15;
robots(1).GazeAmplitude      = deg2rad(80);
robots(1).GazeAngularSpeed   = 2*pi/6;   % 6 s per full left-right-left cycle
robots(1).GazePhase0         = 0;

robots(2).Position          = [-8; 4];
robots(2).Covariance        = diag([0.05, 0.05]);
robots(2).DetectionNoiseStd = deg2rad(1.5);
robots(2).Rate               = 3;
robots(2).Heading           = 0;
robots(2).FOV               = deg2rad(130);
robots(2).Range             = 10;
robots(2).GazeAmplitude      = deg2rad(80);
robots(2).GazeAngularSpeed   = 2*pi/6;
robots(2).GazePhase0         = pi/2;

robots(3).Position          = [-3; -6];
robots(3).Covariance        = diag([0.01, 0.01]);
robots(3).DetectionNoiseStd = deg2rad(0.4);
robots(3).Rate               = 5;
robots(3).Heading           = 0;
robots(3).FOV               = deg2rad(110);
robots(3).Range             = 10;
robots(3).GazeAmplitude      = deg2rad(80);
robots(3).GazeAngularSpeed   = 2*pi/6;
robots(3).GazePhase0         = pi;

robots(4).Position          = [-3; 6];
robots(4).Covariance        = diag([0.03, 0.03]);
robots(4).DetectionNoiseStd = deg2rad(1.25);
robots(4).Rate               = 4;
robots(4).Heading           = 0;
robots(4).FOV               = deg2rad(140);
robots(4).Range             = 10;
robots(4).GazeAmplitude      = deg2rad(80);
robots(4).GazeAngularSpeed   = 2*pi/6;
robots(4).GazePhase0         = 3*pi/2;

robots(5).Position          = [-5.5; 0];
robots(5).Covariance        = diag([0.02, 0.02]);
robots(5).DetectionNoiseStd = deg2rad(1.0);
robots(5).Rate               = 3;
robots(5).Heading           = 0;
robots(5).FOV               = deg2rad(120);
robots(5).Range             = 12;
robots(5).GazeAmplitude      = deg2rad(80);
robots(5).GazeAngularSpeed   = 2*pi/6;
robots(5).GazePhase0         = 8*pi/5;

T_total = 15; % seconds

%% Ball motion: identical scripted kicks to RUN_TRACK_BALL_STATIONARY_ROBOTS
kickTimes      = [1.5, 3.5, 6.5, 10];
kickVelocities = [-0.75, -3.875; ...
                   -3.917,  3.375; ...
                    5.667,  1.43; ...
                   -1.9,   -1.38];

%% Opponent motion: smooth random walk (true path only -- they're targets
% being tracked, not sensors, so there's no believed-vs-true pose split
% to model for them, unlike RUN_MOVING_ROBOTS_STATIONARY_BALL's robots).

cfg   = fieldConfig();
dtSim = 0.02;               % s, fine grid the opponents' true walk is simulated on
tGrid = 0:dtSim:T_total;
Nt    = numel(tGrid);

% All five start on the OPPOSITION's half (x>0) -- our robots are all on
% x<0 above -- then wander freely (STEPPLAYERWALK only reflects off the
% full field boundary, not the halfway line, so they can cross over).
oppStarts(1) = struct('Position', [4, -5], 'Heading', deg2rad(60));
oppStarts(2) = struct('Position', [5, 5],  'Heading', deg2rad(-120));
oppStarts(3) = struct('Position', [8, 0],  'Heading', deg2rad(180));
oppStarts(4) = struct('Position', [3, -2], 'Heading', deg2rad(90));
oppStarts(5) = struct('Position', [3, 2],  'Heading', deg2rad(-90));

s = rng;    % Save random seed
rng(42);    % Set random seed (covers opponent walks + measurement noise below)

opponents = struct([]);
for o = 1:numel(oppStarts)
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

    opponents(o).TruePosition = posHist;
    opponents(o).TrueVelocity = velHist;
end

%% Build one independent event queue + run one independent filter per target

numOpp  = numel(oppStarts);
numTeam = numel(robots);

targetNames = [{'ball'}, ...
    arrayfun(@(o) sprintf('opp%d', o), 1:numOpp, 'UniformOutput', false), ...
    arrayfun(@(j) sprintf('team%d', j), 1:numTeam, 'UniformOutput', false)];
numTargets = numel(targetNames);

initialSystems = cell(1, numTargets);
targetQueues   = cell(1, numTargets);

% --- ball (target 1): same as RUN_TRACK_BALL_STATIONARY_ROBOTS ---

ballQueue = Event.empty;
for r = 1:numel(robots)
    for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
        m = buildMeasurement(robots(r), r, t, 'ball', verbosity);
        ballQueue(end + 1) = m; %#ok<SAGROW>
    end
end
ballQueue = sort(ballQueue);

systemBall = SystemBall();
initialSystems{1} = systemBall;

nextKick = 1;
for k = 1:length(ballQueue)
    [ballQueue(k), systemBall] = ballQueue(k).process(systemBall);
    while nextKick <= size(kickVelocities, 1) && ballQueue(k).time >= kickTimes(nextKick)
        systemBall.x_sim(3:4) = systemBall.x_sim(3:4) + kickVelocities(nextKick, :).';
        nextKick = nextKick + 1;
    end
end
targetQueues{1} = ballQueue;

% --- opponents (targets 2 .. 1+numOpp): every robot detects every opponent ---

for o = 1:numOpp
    idx = 1 + o;
    oppQueue = Event.empty;
    for r = 1:numel(robots)
        for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
            m = buildMeasurement(robots(r), r, t, targetNames{idx}, verbosity);
            oppQueue(end + 1) = m; %#ok<SAGROW>
        end
    end
    oppQueue = sort(oppQueue);

    systemOpp = SystemBall();
    systemOpp.x_sim = [opponents(o).TruePosition(:, 1); opponents(o).TrueVelocity(:, 1)];
    initialSystems{idx} = systemOpp;

    for k = 1:length(oppQueue)
        [oppQueue(k), systemOpp] = oppQueue(k).process(systemOpp);
        % Overwrite with the true, continuously-curving random-walk state
        % at this exact time -- SYSTEMBALL's own (constant-velocity)
        % dynamics would otherwise just extrapolate a straight line
        % between events, same idea as the ball's kicks above.
        kk = min(max(round(oppQueue(k).time/dtSim) + 1, 1), Nt);
        systemOpp.x_sim = [opponents(o).TruePosition(:, kk); opponents(o).TrueVelocity(:, kk)];
    end
    targetQueues{idx} = oppQueue;
end

% --- teammates (targets 2+numOpp .. end): every OTHER robot detects robot j ---
% Stationary and exact, so no per-event override is needed at all --
% SYSTEMBALL's own zero-velocity dynamics keep x_sim exactly put.

for j = 1:numTeam
    idx = 1 + numOpp + j;
    teamQueue = Event.empty;
    for r = 1:numel(robots)
        if r == j
            continue % a robot doesn't detect itself
        end
        for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
            m = buildMeasurement(robots(r), r, t, targetNames{idx}, verbosity);
            teamQueue(end + 1) = m; %#ok<SAGROW>
        end
    end
    teamQueue = sort(teamQueue);

    systemTeam = SystemBall();
    systemTeam.x_sim = [robots(j).Position(:); 0; 0];
    initialSystems{idx} = systemTeam;

    for k = 1:length(teamQueue)
        [teamQueue(k), systemTeam] = teamQueue(k).process(systemTeam);
    end
    targetQueues{idx} = teamQueue;
end

rng(s);     % Restore random seed

mergedEvents = targetQueues{1};
for n = 2:numTargets
    mergedEvents = [mergedEvents, targetQueues{n}]; %#ok<AGROW>
end
mergedEvents = sort(mergedEvents);

%% Plots
% Five windows: the 4 (state) x N (target) true/estimate/+-3sigma grid
% is split across two figures -- one for the 5 opponents, one for the
% ball + 5 teammates -- since all 11 together made one figure too wide
% to read; a separate combined sigma-decay figure (all targets overlaid
% there, since that one doesn't get cluttered the way overlaying
% true/estimate/fill bands would), a field-trajectory overview, and the
% interactive step-through tool.

oppIdx      = 2:(1 + numOpp);
ballTeamIdx = [1, (2 + numOpp):numTargets];

plotStatesGrid(targetQueues(oppIdx), targetNames(oppIdx), 1, 'Opponents');
plotStatesGrid(targetQueues(ballTeamIdx), targetNames(ballTeamIdx), 2, 'Ball + teammates');
plotSigma(targetQueues, targetNames, 3);
plotFieldTrajectory(targetQueues, targetNames, robots);
stepThroughMultiTargetTracking(mergedEvents, targetQueues, targetNames, initialSystems, robots);

%% Local functions

function m = buildMeasurement(robot, robotID, t, targetName, verbosity)
%BUILDMEASUREMENT One MEASUREMENTBALLBEARING instance for one (robot,
%   time, target) combination -- robots don't move POSITION here, so
%   TrueObserverPosition/Heading are just the same fixed values as the
%   believed ObserverPosition (see RUN_TRACK_BALL_STATIONARY_ROBOTS).
%   TrueObserverGazeOffset is the one thing that isn't fixed: it's the
%   robot's head-pan sweep, evaluated in closed form at this exact
%   measurement time (no simulation loop needed, since it doesn't depend
%   on anything but time itself).

gazeOffset = robot.GazeAmplitude * sin(robot.GazePhase0 + robot.GazeAngularSpeed*t);

m = MeasurementBallBearing();
m.time                    = t;
m.ObserverPosition        = robot.Position;
m.ObserverCovariance      = robot.Covariance;
m.DetectionNoiseStd       = robot.DetectionNoiseStd;
m.TrueObserverPosition    = robot.Position;
m.TrueObserverHeading     = robot.Heading;
m.TrueObserverGazeOffset  = gazeOffset;
m.ObserverFOV             = robot.FOV;
m.ObserverRange           = robot.Range;
m.RobotID                 = robotID;
m.TargetName              = targetName;
m.needToSimulate          = true;
m.verbosity               = verbosity;
m.updateMethod            = 'BFGSTrustSqrt'; % see RUN_TRACK_BALL_STATIONARY_ROBOTS for why not 'affine'/'NewtonTrustEig'
end

function h = plotStatesGrid(targetQueues, targetNames, fig, figTitle)
%PLOTSTATESGRID 4 (state) x N (target) grid: one column per tracked
%   thing (ball, opp1, opp2, ..., team1, team2, ...), each cell true/
%   estimate/+-3sigma for that state -- same content as the
%   single-target scripts' own PLOTFIGURE left column, just laid out
%   side by side instead of one figure per target. FIGTITLE (optional)
%   labels the whole window, since opponents and ball/teammates are now
%   split across two separate figures rather than one wide grid.

if nargin < 4
    figTitle = '';
end

numTargets = numel(targetQueues);
stateNames = {'x position [m]', 'y position [m]', 'x velocity [m/s]', 'y velocity [m/s]'};
n_sigma = 3;

hf = figure(fig); clf(fig);
hf.Position = [50, 50, 230*numTargets, 760];

ax = gobjects(4, numTargets);
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
legend(ax(1, 1), {'true', 'estimate', '\pm3\sigma'}, 'Location', 'best')
if ~isempty(figTitle)
    sgtitle(hf, figTitle)
end

h.hf = hf;
h.ax = ax;
ax(1, 1).XLimitMethod = 'tight';
h.link = linkprop(ax(:), {'XLim'});
end

function h = plotSigma(targetQueues, targetNames, fig)
%PLOTSIGMA Marginal standard deviations for all targets overlaid
%   (colour-coded), one subplot per state -- kept separate from
%   PLOTSTATESGRID since overlaying true/estimate/+-3sigma bands for
%   several targets on the same axes would be cluttered, but overlaying
%   just the sigma lines isn't.

numTargets = numel(targetQueues);
targetColors = lines(numTargets);
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
legend(ax(1), legendHandles, targetNames, 'Location', 'best')
title(ax(1), 'Marginal standard deviations (all targets)', 'FontSize', 14)

h.hf = hf;
h.ax = ax;
ax(1).XLimitMethod = 'tight';
h.link = linkprop(ax, {'XLim'});
end

function plotFieldTrajectory(targetQueues, targetNames, robots)
%PLOTFIELDTRAJECTORY Top-down view of every target's true and estimated
%   path, plus each stationary robot's FOV cone, reusing DRAWSOCCERFIELD
%   and DRAWPLAYER.

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

targetColors = lines(numel(targetQueues));
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
title(ax, 'True vs. estimated paths -- ball, opponents, and teammates', 'Color', 'w');
end
