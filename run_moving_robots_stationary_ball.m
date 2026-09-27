% RUN_MOVING_ROBOTS_STATIONARY_BALL Test SYSTEMBALL/MEASUREMENTBALLBEARING
%   with the roles reversed from RUN_TRACK_BALL_STATIONARY_ROBOTS: the
%   ball sits still (slightly off centre), and this time it's the four
%   robots that move, each doing its own smooth random walk around the
%   field (reusing CREATEPLAYER/STEPPLAYERWALK's OU-filtered unicycle
%   model, unchanged). Same event system, same MEASUREMENTBALLBEARING,
%   same centralised-fusion behaviour as before.
%
%   The new part: each robot's BELIEVED pose now drifts away from its
%   TRUE pose over time (an independent OU bias per robot, in position
%   and heading), and the covariance it reports grows correspondingly --
%   via the exact variance ODE of that same OU process (dVar/dt =
%   -2*damping*Var + noiseStd^2), not a fixed guess -- so the reported
%   uncertainty is provably consistent with the drift actually
%   happening. TrueObserverPosition/TrueObserverHeading (ground truth)
%   drive the FOV visibility gate; ObserverPosition/ObserverCovariance/
%   ObserverHeadingVariance (the belief) drive the shared measurement
%   itself -- see MEASUREMENTBALLBEARING.

clc;
clear all;

% Event verbosity
verbosity = 1;  % 0: silent, 1: dots, 2: summary, 3: iter

% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Ball: stationary, slightly off centre (same spot RUN_TEST_YOLO uses)

ballPosition = [1.5, 0.8];

%% Robots: smooth random walk, with a drifting belief about their own pose
% Cleared first: ROBOTS is built field-by-field below, and a leftover
% ROBOTS from a previous RUN_TRACK_BALL_STATIONARY_ROBOTS run in this
% session (with just Position/Covariance/etc., no .t history) would
% otherwise leave STEPTHROUGHBALLTRACKING treating these as static.
clear robots robotStarts robotSpecs

cfg = fieldConfig();

T_total = 15;    % seconds
dtSim   = 0.02;  % s, fine grid the robots' true walk and belief drift are simulated on
tGrid   = 0:dtSim:T_total;
Nt      = numel(tGrid);

robotStarts(1) = struct('Position', [-6, -3], 'Heading', deg2rad(30));
robotStarts(2) = struct('Position', [6, 3],   'Heading', deg2rad(-150));
robotStarts(3) = struct('Position', [-6, 4],  'Heading', deg2rad(-60));
robotStarts(4) = struct('Position', [6, -4],  'Heading', deg2rad(120));

robotSpecs(1) = struct('DetectionNoiseStd', deg2rad(0.75), 'FOV', deg2rad(100), 'Range', 15, 'Rate', 2);
robotSpecs(2) = struct('DetectionNoiseStd', deg2rad(1.5),  'FOV', deg2rad(130), 'Range', 10, 'Rate', 3);
robotSpecs(3) = struct('DetectionNoiseStd', deg2rad(0.4),  'FOV', deg2rad(110), 'Range', 10, 'Rate', 5);
robotSpecs(4) = struct('DetectionNoiseStd', deg2rad(1.25), 'FOV', deg2rad(140), 'Range', 10, 'Rate', 4);

% Believed-pose drift: independent OU bias per robot, in position (m) and
% heading (rad), same style as SYSTEMBALL's own process noise and the
% earlier stashed 3D multi-robot design's ErrX/ErrY/ErrHeading.
dampPos      = 0.3;          % 1/s
noisePosStd  = 0.03;         % m per sqrt(s)
dampHead     = 0.3;          % 1/s
noiseHeadStd = deg2rad(1);   % rad per sqrt(s)

s = rng;    % Save random seed
rng(42);    % Set random seed (covers robot walk + belief drift + measurement noise below)

for r = 1:numel(robotStarts)
    player = createPlayer(robotStarts(r).Position, robotStarts(r).Heading, [0.1 0; 0 0.1]);

    errX = 0; errY = 0; errH = 0;
    varX = 0; varY = 0; varH = 0;

    truePos      = zeros(2, Nt);
    trueHead     = zeros(1, Nt);
    believedPos  = zeros(2, Nt);
    believedHead = zeros(1, Nt); % for display only (see below) -- the bearing measurement itself never uses a heading value, only ObserverHeadingVariance
    believedCov  = zeros(3, Nt); % [varX; varY; varH]

    for kk = 1:Nt
        lookHeading = player.Heading + player.GazeOffset; % walking heading + independent gaze sweep, as in the 2D single-player demo

        truePos(:, kk)      = player.Position(:);
        trueHead(kk)        = lookHeading;
        believedPos(:, kk)  = player.Position(:) + [errX; errY];
        believedHead(kk)    = lookHeading + errH;
        believedCov(:, kk)  = [varX; varY; varH];

        if kk < Nt
            player = stepPlayerWalk(player, cfg, dtSim);

            errX = errX + dtSim*(-dampPos*errX)  + noisePosStd*sqrt(dtSim)*randn();
            errY = errY + dtSim*(-dampPos*errY)  + noisePosStd*sqrt(dtSim)*randn();
            errH = errH + dtSim*(-dampHead*errH) + noiseHeadStd*sqrt(dtSim)*randn();

            % Exact variance ODE of the same OU process: dVar/dt = -2*damping*Var + noiseStd^2
            varX = varX + dtSim*(-2*dampPos*varX  + noisePosStd^2);
            varY = varY + dtSim*(-2*dampPos*varY  + noisePosStd^2);
            varH = varH + dtSim*(-2*dampHead*varH + noiseHeadStd^2);
        end
    end

    robots(r).t                = tGrid;
    robots(r).TruePosition     = truePos;
    robots(r).TrueHeading      = trueHead;
    robots(r).BelievedPosition = believedPos;
    robots(r).BelievedHeading  = believedHead;
    robots(r).BelievedCov      = believedCov;
    robots(r).DetectionNoiseStd = robotSpecs(r).DetectionNoiseStd;
    robots(r).FOV                = robotSpecs(r).FOV;
    robots(r).Range              = robotSpecs(r).Range;
    robots(r).Rate                = robotSpecs(r).Rate;
end

%% Create event queue
% As in RUN_TRACK_BALL_STATIONARY_ROBOTS: each robot's own periodic
% measurement stream, merged and time-sorted. Believed pose/covariance
% for each measurement are sampled from that robot's precomputed history
% at the nearest sample to this measurement's time.

event_queue = Event.empty;

for r = 1:numel(robots)
    for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
        kk = min(max(round(t/dtSim) + 1, 1), Nt);

        measurement = MeasurementBallBearing();
        measurement.time                    = t;
        measurement.ObserverPosition        = robots(r).BelievedPosition(:, kk);
        measurement.ObserverCovariance      = diag(robots(r).BelievedCov(1:2, kk));
        measurement.ObserverHeadingVariance = robots(r).BelievedCov(3, kk);
        measurement.DetectionNoiseStd       = robots(r).DetectionNoiseStd;
        measurement.TrueObserverPosition    = robots(r).TruePosition(:, kk);
        measurement.TrueObserverHeading     = robots(r).TrueHeading(kk);
        measurement.ObserverFOV             = robots(r).FOV;
        measurement.ObserverRange           = robots(r).Range;
        measurement.RobotID                 = r;
        measurement.needToSimulate          = true;
        measurement.verbosity               = verbosity;
        measurement.updateMethod            = 'BFGSTrustSqrt'; % see RUN_TRACK_BALL_STATIONARY_ROBOTS for why not 'affine'/'NewtonTrustEig'
        event_queue(end + 1) = measurement; %#ok<SAGROW>
    end
end

event_queue = sort(event_queue);

%% Create initial system and run event loop

system = SystemBall();
system.x_sim = [ballPosition(:); 0; 0]; % stationary, off centre (prior mean/covariance stay at SYSTEMBALL's defaults)

for k = 1:length(event_queue)
    [event_queue(k), system] = event_queue(k).process(system);
end

rng(s);     % Restore random seed

plotFigure(event_queue, 1);
plotFieldTrajectory(event_queue, robots);
stepThroughBallTracking(event_queue, robots);

%% Post-processing/plotting
function h = plotFigure(event_queue, fig)
% Identical in structure to RUN_TRACK_BALL_STATIONARY_ROBOTS's own
% PLOTFIGURE -- kept as a local copy since MATLAB scripts can't share
% local functions across files.

N = length(event_queue);
t_hist     = nan(1, N);
x_hist     = nan(4, N);
mu_hist    = nan(4, N);
sigma_hist = nan(4, N);
for k = 1:N
    t_hist(:, k) = event_queue(k).time;
    if event_queue(k).saveSystemState
        x_hist(:, k)     = event_queue(k).system.x_sim;
        mu_hist(:, k)    = event_queue(k).system.density.mean();
        sigma_hist(:, k) = realsqrt(diag(event_queue(k).system.density.cov()));
    end
end

n_sigma = 3;
mu_hist_m = mu_hist - n_sigma*sigma_hist;
mu_hist_p = mu_hist + n_sigma*sigma_hist;

stateNames = {'x position [m]', 'y position [m]', 'x velocity [m/s]', 'y velocity [m/s]'};

hf = figure(fig); clf(fig);
hf.Position = [100, 100, 2*560, 2*420];

ax = gobjects(4, 2);
for i = 1:4
    ax(i, 1) = subplot(4, 2, 2*i - 1, 'Parent', hf);
    hold(ax(i, 1), 'on')
    plot(ax(i, 1), t_hist, x_hist(i, :), 'r')
    plot(ax(i, 1), t_hist, mu_hist(i, :), 'Color', [0.1 0.7 0.2])
    fill(ax(i, 1), [t_hist, fliplr(t_hist)], [mu_hist_m(i, :), fliplr(mu_hist_p(i, :))], [0.1 0.7 0.2], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold(ax(i, 1), 'off')
    xlabel(ax(i, 1), 'Time [s]')
    ylabel(ax(i, 1), stateNames{i})
    legend(ax(i, 1), {'true', 'estimate', '\pm3\sigma'})
    grid(ax(i, 1), 'on')

    ax(i, 2) = subplot(4, 2, 2*i, 'Parent', hf);
    semilogy(ax(i, 2), t_hist, sigma_hist(i, :))
    xlabel(ax(i, 2), 'Time [s]')
    ylabel(ax(i, 2), ['\sigma, ' stateNames{i}])
    grid(ax(i, 2), 'on')
end
title(ax(1, 1), 'State estimates', 'FontSize', 14)
title(ax(1, 2), 'Marginal standard deviations', 'FontSize', 14)

h.hf = hf;
h.ax = ax;
ax(1, 1).XLimitMethod = 'tight';
h.link = linkprop(ax(:), {'XLim'});
end

function plotFieldTrajectory(event_queue, robots)
%PLOTFIELDTRAJECTORY Top-down view of the (stationary) ball's estimate
%   next to each robot's true walking path, reusing DRAWSOCCERFIELD.

N = length(event_queue);
x_hist  = nan(4, N);
mu_hist = nan(4, N);
for k = 1:N
    if event_queue(k).saveSystemState
        x_hist(:, k)  = event_queue(k).system.x_sim;
        mu_hist(:, k) = event_queue(k).system.density.mean();
    end
end

ax = drawSoccerField(fieldConfig());
hTrue = plot(ax, x_hist(1, :), x_hist(2, :), 'o', 'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
hEst  = plot(ax, mu_hist(1, :), mu_hist(2, :), '--', 'Color', [0.1 0.7 0.2], 'LineWidth', 1.5);

robotColors = lines(numel(robots));
hRobotPaths = gobjects(1, numel(robots));
robotLabels = cell(1, numel(robots));
for r = 1:numel(robots)
    hRobotPaths(r) = plot(ax, robots(r).TruePosition(1, :), robots(r).TruePosition(2, :), ...
        '-', 'Color', robotColors(r, :), 'LineWidth', 1);
    plot(ax, robots(r).TruePosition(1, 1), robots(r).TruePosition(2, 1), 'o', ...
        'MarkerFaceColor', robotColors(r, :), 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    robotLabels{r} = sprintf('R%d path', r);
end

legend(ax, [hTrue, hEst, hRobotPaths], [{'true ball', 'estimate'}, robotLabels], ...
    'TextColor', 'w', 'Location', 'best');
title(ax, 'Ball estimate vs. robot walking paths', 'Color', 'w');
end
