function p = trackerParamsFill(params)
%TRACKERPARAMSFILL Tracker params struct (as built in run_5v5_decentralised.m
%   :312-323) from the eval params; missing entries take the script's values.
%   Raw names: gateSigma, mergeSigma, oneToOne, mergeMaxStd, spawnPosStd,
%   maxMisses, expectedMargin, selfExclusion, kappaBall, ballAccelStd,
%   robotAccelStd, verbosity. If derived fields (gateThreshold,
%   mergeThreshold) are already present they are kept.
P = @(n, d) evalParamOr(params, n, d);
p.oneToOne       = P('oneToOne', true);
p.mergeMaxStd    = P('mergeMaxStd', 1.0);
p.spawnPosStd    = P('spawnPosStd', 3);
p.maxMisses      = P('maxMisses', 5);
p.expectedMargin = P('expectedMargin', 0.8);
p.selfExclusion  = P('selfExclusion', 1.0);
p.kappaBall      = P('kappaBall', 0.2);
p.ballAccelStd   = P('ballAccelStd', 2.5);
p.robotAccelStd  = P('robotAccelStd', 0.5);
p.verbosity      = P('verbosity', 0);
p.spawnAnalytic  = P('spawnAnalytic', false);
p.spawnInflate   = P('spawnInflate', 4);
p.assocLogDet    = P('assocLogDet', false);
p.mergeMinFusions = P('mergeMinFusions', 0);
p.lostStd        = P('lostStd', inf);
p.lostTime       = P('lostTime', inf);
p.minSpawnRange  = P('minSpawnRange', 0);
p.obsCovInflate  = [P('obsCovInflate', 1), P('obsCovInflateBall', 1)]; % [robot ball]
p.spawnAnalyticBall = P('spawnAnalyticBall', false);
p.spawnVelStd    = P('spawnVelStd', 2.5);
% Ball / robot restart study options (tools/ballstudy), all default OFF
p.ballSingle     = P('ballSingle', false);     % A: at most one live ball track
p.ballLostStd    = P('ballLostStd', inf);      % B: delete unseen ball track when pos std > this (m)
p.ballLostTime   = P('ballLostTime', inf);     % B: ... or unseen for this long (s)
p.ballResetN     = P('ballResetN', inf);       % C: consecutive scans whose ball detections all fail the gate -> reset
p.ballResetD2    = P('ballResetD2', inf);      % C: immediate reset if min Mahalanobis^2 of failing ball dets exceeds this
p.ballResetVmax  = P('ballResetVmax', inf);    % C: immediate reset if implied speed to the track exceeds this (m/s)
p.ballResetTimes = P('ballResetTimes', []);    % D: referee/out-of-play times (EXTERNAL info)
p.robotResetEvents = P('robotResetEvents', []);% Nx3 [t x y] robot removal events (EXTERNAL info)
p.robotResetGate = P('robotResetGate', 1.5);   % m
p.spawnMaxRangeStd = P('spawnMaxRangeStd', inf); % never spawn from a detection with range std above this (m)
p.spawnMaxArea     = P('spawnMaxArea', inf);     % ... or implied position-covariance area pi*sr*st above this (m^2)
p.spawnPending     = P('spawnPending', false);   % park such detections; confirm by 2nd robot (ray intersection)
p.spawnPendWindow  = P('spawnPendWindow', 0.5);  % s
p.spawnPolicyRobots = P('spawnPolicyRobots', false); % apply the spawn policy to robot-class detections too
p.ballReplaceStd   = P('ballReplaceStd', inf);   % good ball det replaces ball tracks with pos std above this (m)
gs = P('gateSigma', 3);
ms = P('mergeSigma', 3);
p.gateThreshold  = P('gateThreshold', chi2inv(2*normcdf(gs) - 1, 3));
if isnan(ms)
    p.mergeThreshold = NaN;
else
    p.mergeThreshold = P('mergeThreshold', chi2inv(2*normcdf(ms) - 1, 2));
end
end
