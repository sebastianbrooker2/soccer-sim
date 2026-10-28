function b = baselineDefaults(params)
%BASELINEDEFAULTS Baseline tracker parameters. params.baseline overrides.
%   params.baseline.mode:
%     'faithful' (default)  re-implementation of NUbots_K1 RobotLocalisation /
%                           BallLocalisation behaviour (see runBaselineTracker).
%     'tuned'               strongest fair per-robot competitor: R derived per
%                           detection, Mahalanobis (chi2 99%) gating, tracks age
%                           out after maxAge, one-to-one assignment.
%   Mode-dependent defaults are applied first, then explicit params.baseline
%   fields override them.
mode = 'faithful';
ov = struct();
if nargin > 0 && isstruct(params) && isfield(params, 'baseline') && isstruct(params.baseline)
    ov = params.baseline;
    if isfield(ov, 'mode') && ~isempty(ov.mode), mode = ov.mode; end
end
% --- common (NUbots config: Robot/BallLocalisation.yaml) ---
b = struct('mode', mode, 'R', 0.1, 'Qball', 0.01, 'QoppPos', 0.1, 'QoppVel', 0.01, ...
    'gate', 0.5, ...              % association_distance (robots), m
    'maxMisses', 25, ...          % max_missed_count: deleted when missed_count > maxMisses
    'mergeDist', 0.5, 'selfExclude', 0, 'fieldMargin', 1.0, ...
    'P0pos', 1e-3, 'P0vel', 1e-3, 'maxAge', inf, ...
    'proximityDelete', true, ...  % NUbots MAIN RobotLocalisation.cpp:256-261 std::any_of proximity removal (working)
    'k1ProximityBug', false, ...  % true = NUbots_K1 branch (RobotLocalisation.cpp:256-270): any_of commented out, inner-loop `continue` is dead code -> NO proximity removal
    'ballInit', 'config', 'ballInitMean', [5; 0], 'ballInitCov', 1e-3, ...
    'ballAccept', 2, 'ballMaxRej', 25, ...
    'useTeamBall', true, 'teamMsgPeriod', 1.0, 'teamBallRecency', 3.0, ...
    'teamGuessDefaultTimer', 2.0, 'teamBallStd', 0.5, 'teamBallMaxAge', 5.0, ...
    'measurement', 'nubots', ...  % faithful: 'nubots' (raw YOLO box -> ground projection, nubotsMeasurement) | 'legacy' (packet range/bearing)
    'nubotsOpt', struct('confBall', 0.6, 'confRobot', 0.2, 'quantise', true), ...
    'chi2Gate', 9.2103, ...       % chi2inv(0.99, 2); tuned mode
    'ballReacquire', 3);          % tuned: consecutive gated-out ball dets before re-init
if strcmp(mode, 'tuned')
    % (tuned always uses packet range/bearing + per-detection covariance)
    b.maxAge = 3; b.P0pos = 0.1; b.P0vel = 1.0; b.useTeamBall = false; b.ballInit = 'detection';
    b.selfExclude = 0.5;
end
f = fieldnames(ov);
for i = 1:numel(f), b.(f{i}) = ov.(f{i}); end
if b.k1ProximityBug, b.proximityDelete = false; end
if isscalar(b.R), b.R = b.R*eye(2); end
b.ballInitMean = b.ballInitMean(:);
end
