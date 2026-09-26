% RUNSIMULATOR Entry point for the soccer simulator.
%   Run this script to open a 2D visualisation of the field with a
%   player walking a smooth, curving path while its gaze sweeps
%   independently of its walking direction. Close the figure window to
%   stop.
%
%   Restructured to follow RUN_BALLISTIC.M's pattern: build a queue of
%   event times up front, run a seeded loop that steps the system and
%   saves its history, then animate/plot from that saved history
%   afterwards -- rather than stepping and drawing live in one loop.
%
%   NOTE: RUN_BALLISTIC.M dispatches through EVENT/MEASUREMENT objects
%   acting on a SYSTEMBASE (SystemBallistic), sorted into an event queue
%   and processed with event_queue(k).process(system). There is no such
%   class for this player/ball model yet -- STEPPLAYERWALK and
%   COMPUTEBALLDETECTION are plain functions on plain structs, not a
%   SystemBase/Measurement pair. Matching RUN_BALLISTIC.M exactly would
%   need two new files (not created here, since only this file was to
%   change):
%     - @SystemPlayerWalk < SystemBase, wrapping PLAYER (and BALL) state
%       and exposing STEPPLAYERWALK's motion model through
%       dynamics()/predict();
%     - @MeasurementBallDetection < Measurement, wrapping
%       COMPUTEBALLDETECTION's simulate()/logLikelihood().
%   Until those exist, the "event queue" below is just a vector of
%   times, and the loop calls STEPPLAYERWALK/COMPUTEBALLDETECTION
%   directly instead of EVENT.PROCESS.

verbosity = 1; % 0: silent, 1: dots per step, 2: summary at the end

% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Create event queue
% Note: as in RUN_BALLISTIC.M, the event queue can be the entire time
%       sequence up front, since we know all the event times in advance.

dt      = 0.03; % seconds per simulation step
T_total = 60;   % total simulated time, s (matches RUN_BALLISTIC.M's horizon)

t_queue = 0:dt:T_total;

%% Create initial system

cfg = fieldConfig();

player = createPlayer([-3, 1], deg2rad(20), [0.3 0.05; 0.05 0.15]); % spawn in a player, initial positoin and covariance is input

ball  = createBall([1.5, 0.8], cfg);   % create ball is stationary for now, just off the centre spot near the centre circle
ball.Radius = ball.Radius * 3;         % drawn 3x the regulation radius so it's easy to see

%% Run event loop

N = numel(t_queue);
pos_hist        = nan(2, N);
heading_hist    = nan(1, N);
gaze_hist       = nan(1, N);
detected_hist   = false(1, N);
confidence_hist = nan(1, N);

s = rng;    % Save random seed
rng(42);    % Set random seed
for k = 1:N
    pos_hist(:, k)  = player.Position(:);
    heading_hist(k) = player.Heading;
    gaze_hist(k)    = player.GazeOffset;

    det = computeBallDetection(player, ball);
    detected_hist(k)   = det.Detected;
    confidence_hist(k) = det.Confidence;

    if verbosity >= 1
        fprintf('.');
    end

    if k < N
        player = stepPlayerWalk(player, cfg, dt);
    end
end
rng(s);     % Restore random seed

if verbosity >= 1
    fprintf('\n');
end
if verbosity >= 2
    fprintf('Ball detected in %d/%d steps (%.1f%%)\n', sum(detected_hist), N, 100*mean(detected_hist));
end

%% Post-processing/plotting

animatePlayerHistory(cfg, player, ball, pos_hist, heading_hist, gaze_hist, detected_hist, confidence_hist, dt);

% test

%% Local functions

function animatePlayerHistory(cfg, player, ball, pos_hist, heading_hist, gaze_hist, detected_hist, confidence_hist, dt)
%ANIMATEPLAYERHISTORY Replay a saved player/detection history on the field.
%   Draws from POS_HIST/HEADING_HIST/GAZE_HIST/DETECTED_HIST/
%   CONFIDENCE_HIST (saved during the event loop above) rather than
%   recomputing STEPPLAYERWALK/COMPUTEBALLDETECTION, so the animation
%   matches exactly what was simulated -- COMPUTEBALLDETECTION adds
%   random jitter each call, so recomputing it here would draw fresh,
%   different noise instead of replaying the saved run.
%
%   Duplicates UPDATEBALLDETECTION's show/hide logic inline for that
%   reason; a cleaner fix would be a new UPDATEBALLDETECTIONFROMRESULT(H,
%   DET) that takes a precomputed DET struct (not created here, per
%   instructions to only change this file).

    ax = drawSoccerField(cfg);

    frame = player;
    frame.Position   = pos_hist(:, 1)';
    frame.Heading    = heading_hist(1);
    frame.GazeOffset = gaze_hist(1);
    h = drawPlayer(ax, frame);

    hBall = drawBall(ax, ball);

    fig = ancestor(ax, 'figure');
    N = size(pos_hist, 2);

    for k = 1:N
        if ~isvalid(fig)
            break
        end

        frame.Position   = pos_hist(:, k)';
        frame.Heading    = heading_hist(k);
        frame.GazeOffset = gaze_hist(k);
        updatePlayer(h, frame);

        if detected_hist(k)
            set(hBall.DetectionRing, 'Visible', 'on');
            set(hBall.Label, 'Visible', 'on', 'String', sprintf('ball %.0f%%', 100*confidence_hist(k)));
        else
            set(hBall.DetectionRing, 'Visible', 'off');
            set(hBall.Label, 'Visible', 'off');
        end

        drawnow limitrate;
        pause(dt);
    end

end
