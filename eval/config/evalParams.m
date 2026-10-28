function p = evalParams()
%EVALPARAMS Single source of all evaluation parameters (see eval/SPEC.md).
%   Defaults come from run_5v5_decentralised.m; K1 values from the K1 camera
%   investigation (NUbots VisualMesh / head limits).

%% Field / teams
p.field.length = 14;   p.field.width = 9;      % m, origin at centre, x fwd, y left
p.field.goalX  = 7;    p.field.goalHalfWidth = 1.3;
p.ids.ball = 0;  p.ids.ours = 1:3;  p.ids.opp = 4:6;
p.teamOf   = [1 1 1 2 2 2];                     % team of ids 1..6
p.observers = 1:3;                              % observing robots

%% Sensor (K1 head camera)
s.FOV         = deg2rad(73.7); % Booster sim K1 head camera: fovy 58 deg, 320x240 (K1_22dof.xml). Real robot FOV still TODO
s.Rate        = 15;            % Hz vision rate. TODO measure on K1 (placeholder)
s.Range       = 11;            % m, VisualMesh max_distance (K1)
s.minRangeBall  = 2.5;         % m, ball detector min range (K1)
s.minRangeRobot = 0.5;         % m, robot detector min range (K1)
s.headYawLimit  = 0.785;       % rad, +-45 deg (K1)
s.headPitchLim  = [-0.3 1.0];  % rad (K1)
s.GazeAmplitude = 0.785;       % rad, search sweep +-0.9 clipped to +-0.785 (K1)
s.GazeSweepAmp  = 0.9;         % rad, unclipped sweep amplitude
s.gazePeriod    = 6;           % s per full sweep cycle, TODO K1 (default from run_5v5_decentralised)
s.GazeAngularSpeed = 2*pi/s.gazePeriod;   % rad/s (derived; makePackets uses it)
s.GazePhase0    = 2*pi*(0:2)/3; % per observer (rad), phase of the sinusoidal sweep
s.fixationTime  = 1.1;         % s per PlanLook (K1)
s.DetectionNoiseStd = deg2rad(1.0);  % bearing std, rad. TODO K1
s.RangeNoiseStd     = 0.30;          % m. TODO K1
s.observerPosStd = 0.1;        % m, self-localisation std (per axis); AR(1) bias in makePackets
s.posCorrTime    = 5;          % s, correlation time of the self-position error
s.headingStd     = deg2rad(2); % rad, self-heading std (folded into bearing noise)
s.ObserverCovariance = s.observerPosStd^2*eye(2); % m^2 (derived, carried in every packet)
s.ObserverHeadingVariance = s.headingStd^2;       % rad^2 (derived)
s.model = 'camera';             % 'cone' (legacy wedge) | 'camera' (projected K1 head camera, makePacketsCamera)
s.camZfar = 20;                % m, camera-model max detection depth (sim detector detected balls out to 14.4 m in det01)
s.pixStd = 1.5;                % px, detection bbox noise (camera model)
s.headStd = deg2rad(0.5);      % rad, measured head-angle noise (camera model)
s.tiltStd = deg2rad(0.5);      % rad, measured trunk roll/pitch (IMU) noise
s.pitchStd = deg2rad(1);       % rad, pitch uncertainty used in range std
s.robotHeight = 0.95;  s.robotWidth = 0.30;  s.ballRadius = 0.107;   % m
s.minRobotPx = 8;              % px, min robot bbox height (ASSUMPTION; sim does not detect robots)
s.fallbackHeadPitch = 0.25;    % rad down, when gt has no head data
s.trunkHeight = 0.55;          % m, when gt has no z
s.confidenceFade = 0.8;        % fraction of FOV/range with full confidence
% YOLO detector sensor (sensor.model='yolo' / opts.packetSource='yolo', makePacketsYolo)
y.minConf = struct('ball', 0.6, 'robot', 0.2);  % (matches detector export thresholds) per-class confidence threshold
y.pixStd = 2;           % px, bbox centre std (only used if y.model = [])
y.sizeStd = 2;          % px, bbox width/height std (only used if y.model = [])
y.rangeInflate = 1;     % multiplier on per-detection range std (TODO measured range model)
y.maxRange = 20;        % m, conversion cutoff
y.expectRange = 8;      % m, packet.Range used by the tracker's expected-visible miss counting
y.headingStd = deg2rad(0.5);  % rad, believed-heading std put in the packets (was 2 deg via sensor.headingStd): makePacketsYolo injects NO heading error beyond headStd, so 2 deg was unsimulated conservatism (-30% ball / -11% opp area on real01)
y.teammateGate = 10;    % px margin around projected teammate box (dropTeammateDetections)
y.model = yoloErrorModel();  % calibrated, size-dependent box error model (tools/yolo); [] = pixStd/sizeStd first-order
s.yolo = y;
p.sensor = s;
p.dropTeammateDetections = true;  % teammate identity is known via the network: do not emit teammate detections as 'robot' class


%% Communication
p.linkDropProb = 0.20;

%% Shared tracker (run_5v5_decentralised.m)
t.gateSigma = 3;  t.oneToOne = true;  t.mergeMaxStd = 1.0;  t.mergeSigma = 3;
t.maxMisses = 5;  t.expectedMargin = 0.8;  t.selfExclusion = 1.0;
t.kappaBall = 0.2;  t.ballAccelStd = 2.0;  t.robotAccelStd = 0.5;  t.spawnPosStd = 3;  % ballAccelStd was 2.5 (real01: area -13%, coverage unchanged); robotAccelStd 0.3/0.4 tested: rmse worse
% Cooperative-tracker improvements (eval/tracker; each knob defaults to the
% original script's behaviour in trackerParamsFill, these are the eval settings).
t.spawnAnalytic = true;  t.spawnInflate = 1;  t.spawnVelStd = 1.0;  % analytic Cartesian first-fusion covariance (robots); 1 m/s vel prior
t.spawnAnalyticBall = true;  % same for the ball: first fusion is deep along the line of sight, tight across it (was a round 3 m prior: long axis ended up ACROSS the ray)
t.minSpawnRange = 1.5;   % m, no new robot track from a detection closer than this (pose-error dominated)
t.lostStd = 4;           % m, delete robot tracks whose position std grows beyond this
t.lostTime = inf;        % s without fusion before deletion (inf = off)
t.obsCovInflate = 2;     % robot-class measurements: sender pose covariance x2 (was 16; real01 tuning: NEES opp 0.9->1.6, inRegion95|track 0.985->0.96, area -53%)
t.obsCovInflateBall = 1;
t.mergeSigma = 2;  t.mergeMaxStd = 0.6;  t.mergeMinFusions = 0;
t.assocLogDet = true;    % association cost d2 + log det S
% Ball handling (tools/ballstudy/README.md; real01: live ball tracks 2.2 -> 1.0, rmse 0.87 -> 0.63 m)
t.ballSingle = true;       % at most one live ball track per copy (keep the best-conditioned)
t.ballLostStd = 8;         % m, delete an unseen ball track whose position std exceeds this
t.ballReplaceStd = 8;      % m, a well-conditioned ball detection replaces a ball track this vague
t.ballResetN = 3;          % scans whose ball detections all fail every ball gate -> restart the ball track
t.spawnMaxRangeStd = 0.6;  % m, ball detections with larger range std never spawn a track by themselves ...
t.spawnPending = true;     % ... they wait as pending hypotheses until a second robot corroborates (ray intersection)
% External-info options (NOT enabled): t.ballResetTimes = [..]; t.robotResetEvents = [t x y; ..] (referee stand-in)
p.tracker = t;

%% Baseline tracker (NUbots, same on K1)
% Names match eval/baseline/baselineDefaults.m (baselineEstimates uses the defaults for prediction)
b.R = 0.1;  b.Qball = 0.01;  b.QoppPos = 0.1;  b.QoppVel = 0.01;   % Q = q*dt
b.gate = 0.5;  b.maxMisses = 25;  b.mergeDist = 0.5;  b.selfExclude = 0.5;
p.baseline = b;

%% Evaluation
p.evalDt = 0.1;
p.confidence = 0.95;
p.radii = [0.25 0.5 1.0];
p.includeTeammates = false;
end
