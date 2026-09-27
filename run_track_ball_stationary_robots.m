% RUN_TRACK_BALL_STATIONARY_ROBOTS Test SYSTEMBALL/MEASUREMENTBALLBEARING.
%   Same shape as RUN_BALLISTIC.M: build an event queue up front, run it
%   through a seeded event loop against one SYSTEMBALL, then plot. Here
%   the "sensors" are several STATIONARY robots at known (but uncertain)
%   positions around the field, each periodically reporting a BEARING
%   (angle) to the ball, not its position -- see MEASUREMENTBALLBEARING
%   for why that makes the estimate's covariance ellipse elongated
%   rather than circular. Every MEASUREMENTBALLBEARING event updates the
%   same SYSTEMBALL.density, regardless of which robot it came from --
%   this is the centralised-fusion behaviour described earlier.

clc;
clear all;

% Enable and run the smoother
runSmoother = false;

% Event verbosity
verbosity = 1;  % 0: silent, 1: dots, 2: summary, 3: iter

% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Define stationary robots (known position, with uncertainty)
% Position/Covariance are the robot's own believed pose and its
% uncertainty (see MEASUREMENTBALLBEARING) -- the robot's dynamics are
% not modelled here at all, only this fixed, known-but-uncertain value.
% Heading/FOV/Range describe the same detection cone as CREATEPLAYER's 2D
% demo (see COMPUTEBALLDETECTION), aimed roughly at the ball's path below.
%
% Cleared first: ROBOTS is built field-by-field below, and MATLAB struct
% arrays don't drop old fields when you do that -- if a previous run in
% this session left a ROBOTS with moving-robot fields (.t/.TruePosition/
% etc., from RUN_MOVING_ROBOTS_STATIONARY_BALL) sitting in the
% workspace, those would otherwise still be there for STEPTHROUGHBALL
% TRACKING to (wrongly) pick up as "this robot moves".
clear robots

robots(1).Position          = [-3; -2];
robots(1).Covariance        = diag([0.02, 0.02]);   % well-localised
robots(1).DetectionNoiseStd = deg2rad(0.75);
robots(1).Rate               = 2;                     % Hz
robots(1).Heading           = deg2rad(30);
robots(1).FOV               = deg2rad(100);
robots(1).Range             = 15;

robots(2).Position          = [5; 3];
robots(2).Covariance        = diag([0.05, 0.05]);   % less well-localised
robots(2).DetectionNoiseStd = deg2rad(1.5);
robots(2).Rate               = 3;
robots(2).Heading           = deg2rad(-135);
robots(2).FOV               = deg2rad(130);
robots(2).Range             = 10;

robots(3).Position          = [1; 5];
robots(3).Covariance        = diag([0.01, 0.01]);
robots(3).DetectionNoiseStd = deg2rad(0.4); % sharpest detector of the three
robots(3).Rate               = 5;
robots(3).Heading           = deg2rad(-45);
robots(3).FOV               = deg2rad(110);
robots(3).Range             = 10;

% Placed along the middle of the t=3.5->6.5 westward "away" leg (see
% kickTimes/kickVelocities below), not at its far end -- so it picks the
% ball up soon after kick2 but well BEFORE kick3, while R1-3 have lost
% it. Catching it early here means only a fraction of one kick's worth
% of unmodelled drift needs correcting on first re-detection, instead of
% two full kicks' worth; the latter was too large a jump for the
% optimiser to bridge in one BFGSTrustSqrt update (it was running out of
% iterations without converging).
robots(4).Position          = [-4; -4];
robots(4).Covariance        = diag([0.03, 0.03]);
robots(4).DetectionNoiseStd = deg2rad(1.25);
robots(4).Rate               = 4;
robots(4).Heading           = deg2rad(-90);
robots(4).FOV               = deg2rad(140);
robots(4).Range             = 10;

T_total = 15; % seconds

% A few sudden velocity changes ("kicks") applied straight to the TRUE
% ball state below, so it doesn't just go in a straight line. SYSTEMBALL
% itself stays exactly constant-velocity -- that's the point of the
% estimator's process model being simpler than the truth, with
% SYSTEMBALL.PROCESSNOISE's process noise absorbing the mismatch -- so
% these kicks are injected here in the run script, not in
% SYSTEMBALL.DYNAMICS.
%
% Kicked to route hand-checked (against each robot's Position/Heading/
% FOV/Range above) clear of all three robots' cones/ranges for a
% stretch, then back into R1's view and loiter there, so sigma should
% grow while it's away and collapse again on re-detection:
%   t=0.0->1.5  (0,0)->(1.5,0.75)   straight line, within R1's cone
%   t=1.5->3.5  ->(2,-6)            south, clears R1's cone en route
%   t=3.5->6.5  ->(-9,-6)           west, out of R2/R3 range, R1 angle
%   t=6.5->10   ->(-2,-1)           back into R1's cone/range
%   t=10->15    nearly stationary around (-2,-1), well inside R1's view
kickTimes      = [1.5, 3.5, 6.5, 10];         % s
kickVelocities = [-0.75, -3.875; ...          % [dvx dvy] added at each kickTimes(i)
                   -3.917,  3.375; ...
                    5.667,  1.43; ...
                   -1.9,   -1.38];

%% Create event queue
% Note: as in RUN_BALLISTIC.M, the event queue can be the entire sequence
%       up front, since we know all the event times in advance. Each
%       robot contributes its own periodic stream of measurements, so
%       the merged queue is naturally asynchronous across robots.

event_queue = Event.empty;

for r = 1:numel(robots)
    for t = 1/robots(r).Rate : 1/robots(r).Rate : T_total
        measurement = MeasurementBallBearing();
        measurement.time                    = t;
        measurement.ObserverPosition        = robots(r).Position;
        measurement.ObserverCovariance      = robots(r).Covariance;
        measurement.DetectionNoiseStd       = robots(r).DetectionNoiseStd;
        % Robots are static here: no true-vs-believed pose split is
        % simulated at all, so ground truth for the FOV gate is just
        % the same fixed values.
        measurement.TrueObserverPosition    = robots(r).Position;
        measurement.TrueObserverHeading     = robots(r).Heading;
        measurement.ObserverFOV             = robots(r).FOV;
        measurement.ObserverRange           = robots(r).Range;
        measurement.RobotID                 = r;
        measurement.needToSimulate          = true;    % Use simulated data (relies on simulated state)
        measurement.verbosity               = verbosity;
        % 'affine' (the class default) linearises the nonlinear bearing
        % just once, about the prior mean -- fine when the prior is
        % close to the truth, but this scenario deliberately runs the
        % ball out of view long enough that it often isn't. BFGSTrustSqrt
        % iteratively re-linearises toward the true MAP instead, the same
        % method RUN_BALLISTIC.M already uses for its own nonlinear
        % MeasurementRADAR. (NewtonTrustEig was tried first but crashes
        % here: it eigendecomposes the TRUE Hessian and takes realsqrt of
        % its eigenvalues, which fails outright if that Hessian isn't
        % positive definite at the point it converges to -- exactly what
        % happens on the first detection after a long, badly-drifted gap.
        % BFGS's Hessian is a rank-2-updated approximation that stays
        % positive definite by construction, so it doesn't hit this.)
        measurement.updateMethod            = 'BFGSTrustSqrt';
        event_queue(end + 1) = measurement; %#ok<SAGROW>
    end
end

% Sort event queue in non-decreasing time order
event_queue = sort(event_queue);

%% Create initial system and run event loop

system = SystemBall();
system.enableSmoother = runSmoother;

% Run event loop
s = rng;    % Save random seed
rng(42);    % Set random seed
nextKick = 1;
for k = 1:length(event_queue)
    [event_queue(k), system] = event_queue(k).process(system);  % Process event

    % Apply any kick whose scheduled time this event has now reached.
    while nextKick <= size(kickVelocities, 1) && event_queue(k).time >= kickTimes(nextKick)
        system.x_sim(3:4) = system.x_sim(3:4) + kickVelocities(nextKick, :).';
        nextKick = nextKick + 1;
    end

    % --- DEBUG: stop at the very first event whose resulting density is
    % corrupted, instead of letting the same warning repeat for the rest
    % of the run. Checks Xi directly (not .mean()), so this doesn't
    % itself trigger the singular-matrix warning.
    Xi_k = system.density.sqrtInfoMat();
    if any(~isfinite(Xi_k(:)))
        fprintf(2, '\n[DEBUG] system.density.Xi is non-finite after event k=%d, t=%.4f, class=%s\n', ...
            k, event_queue(k).time, class(event_queue(k)));
        if isa(event_queue(k), 'MeasurementBallBearing')
            d = event_queue(k).LastDetection;
            fprintf(2, '        RobotID=%d Detected=%d Confidence=%.4f Distance=%.4f AngleError=%.4f\n', ...
                event_queue(k).RobotID, d.Detected, d.Confidence, d.Distance, d.AngleError);
        end
        fprintf(2, '        x_sim = [%s]\n', sprintf('%.4f ', system.x_sim));
        fprintf(2, '        Xi =\n');
        disp(Xi_k);
        error('DEBUG:densityCorrupted', 'Stopping at first corrupted event, see diagnostics above.');
    end
end
rng(s);     % Restore random seed

plotFigure(event_queue, 1);
plotFieldTrajectory(event_queue, robots);
stepThroughBallTracking(event_queue, robots);

%% Run smoother
if runSmoother
    event_queue = event_queue.smooth();
    plotFigure(event_queue, 2);
end

%% Post-processing/plotting
function h = plotFigure(event_queue, fig)

% Get data for plotting from saved system state at each event
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
        sigma_hist(:, k) = realsqrt(diag(event_queue(k).system.density.cov())); % Square root of diagonal of P
    end
end

n_sigma = 3;
mu_hist_m = mu_hist - n_sigma*sigma_hist;
mu_hist_p = mu_hist + n_sigma*sigma_hist;

stateNames = {'x position [m]', 'y position [m]', 'x velocity [m/s]', 'y velocity [m/s]'};

%% Time-series plot
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

% Share common time axis zoom
ax(1, 1).XLimitMethod = 'tight';
h.link = linkprop(ax(:), {'XLim'});
end

function plotFieldTrajectory(event_queue, robots)
%PLOTFIELDTRAJECTORY Top-down view of the true and estimated ball path,
%   plus each stationary robot's FOV cone, reusing DRAWSOCCERFIELD and
%   DRAWPLAYER (the same cone/marker/covariance-ellipse rendering as the
%   2D single-player demo -- each robot here is just a PLAYER-shaped
%   struct that never moves).

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
hTrue = plot(ax, x_hist(1, :), x_hist(2, :), 'r-', 'LineWidth', 1.5);
hEst  = plot(ax, mu_hist(1, :), mu_hist(2, :), '--', 'Color', [0.1 0.7 0.2], 'LineWidth', 1.5);

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

legend(ax, [hTrue, hEst], {'true path', 'estimated path'}, 'TextColor', 'w', 'Location', 'best');
title(ax, 'True vs. estimated ball path', 'Color', 'w');
end
