function params = simParams()
%SIMPARAMS Tunable configuration for the multi-robot ball-detection simulator.
%   PARAMS = SIMPARAMS() returns every parameter used by
%   SIMULATEGROUNDTRUTH and GENERATEDETECTIONS, plus a fixed RNG seed so
%   runs are reproducible. Reuses FIELDCONFIG for field/ball geometry.

    params.Seed  = 42;
    params.Field = fieldConfig();

    params.Dt       = 0.01;   % ground-truth integration step, s
    params.Duration = 30;     % total simulated time, s

    params.NumRobots = 5;

    % --- robot walking behaviour ---
    params.RobotBehavior          = 'walkToBall';  % 'walkToBall' | 'waypoints'
    params.WalkToBallOffsetRadius = 2.0;           % m, robots spread around the ball at this radius
    params.NominalSpeed = 0.8;
    params.MinSpeed     = 0.2;
    params.MaxSpeed     = 1.6;
    params.MaxTurnRate  = deg2rad(90);
    params.HeadingKp     = 2.0;    % turn-rate P-gain on heading error
    params.TurnDamping   = 1.5;    % 1/s, OU relaxation of TurnRate to its P-controlled target
    params.TurnNoiseStd  = 0.3;    % rad/s, OU innovation std on TurnRate
    params.SpeedDamping  = 1.0;    % 1/s, OU relaxation of Speed to its target
    params.SpeedNoiseStd = 0.15;   % m/s, OU innovation std on Speed
    params.WaypointArriveRadius = 0.5; % m

    % --- walking wobble: "small periodic roll/pitch of the body" ---
    params.WobbleRollAmplitude  = deg2rad(3);
    params.WobbleRollFrequency  = 1.6;  % Hz
    params.WobblePitchAmplitude = deg2rad(2);
    params.WobblePitchFrequency = 3.2;  % Hz (~2x roll, like a real walking gait)

    % --- head behaviour ---
    params.HeadHeight       = 0.55;            % m, neck joint height above ground in body frame
    params.CameraOffsetHead = [0.03; 0; 0.02]; % camera position rel. to head joint, in head frame (m)
    params.CameraTiltDown   = deg2rad(5);      % fixed extrinsic tilt of camera below head-forward
    params.HeadYawLimit     = deg2rad(100);    % +/- gimbal limit
    params.HeadPitchLimit   = [deg2rad(-40), deg2rad(25)]; % [down, up] gimbal limit
    params.MaxHeadYawRate   = deg2rad(180);
    params.MaxHeadPitchRate = deg2rad(120);
    params.HeadScanAmplitude = deg2rad(60);
    params.HeadScanFrequency = 0.25;  % Hz
    params.HeadScanPitch     = deg2rad(5);  % nominal head pitch while scanning (down = positive)
    params.HeadTrackMargin   = 1.2;   % track while ball is within this factor of MaxRange/FOV

    % --- camera intrinsics (shared default across robots) ---
    params.ImageWidth  = 640;
    params.ImageHeight = 480;
    params.fx = 500;
    params.fy = 500;
    params.cx = 320;
    params.cy = 240;
    params.DistortionCoeffs = [0 0];    % [k1 k2] radial, [0 0] = disabled

    params.MaxRange = 8;                 % m
    params.PixelNoiseStd         = 1.5;  % px, at close range
    params.PixelNoiseRangeGrowth = 0.15; % extra px std per metre of range

    params.FrameRateBase      = 30;   % Hz, nominal per-robot camera rate
    params.FrameRateJitterStd = 2;    % Hz, per-robot std so rates aren't all identical
    params.RandomFramePhase   = true; % desync robot cameras

    % --- detection probability model ---
    params.DetectionProbMax          = 0.97;
    params.DetectionProbRangeFalloff = 0.5;  % rate p_d decays with range/MaxRange

    % --- occlusion ---
    params.OcclusionEnabled    = true;
    params.RobotCylinderRadius = 0.15; % m
    params.RobotCylinderHeight = 0.9;  % m

    % --- false positives / clutter ---
    params.FalsePositivesEnabled = false;
    params.FalsePositiveRate     = 0.05; % expected false positives per robot per second

    % --- localisation error: believed pose vs. true pose ---
    % Modelled as a mean-reverting (Ornstein-Uhlenbeck) bias per axis, in
    % the same spirit as the smoothed Speed/TurnRate in STEPPLAYERWALK, so
    % it drifts instead of jumping. Only x, y, yaw and head pitch carry a
    % believed bias; robots do not model body roll/pitch wobble at all in
    % their belief (they assume an upright torso), so the true wobble is
    % an additional, unreported source of bearing error.
    params.LocErrorDamping  = [0.3, 0.3, 0.2, 0.2];                       % 1/s, for [x y yaw headPitch]
    params.LocErrorNoiseStd = [0.02, 0.02, deg2rad(1.5), deg2rad(1.0)];   % OU innovation std

    % Reported covariance = ReportedCovScale * (true OU stationary variance).
    % 1 = consistent, <1 = overconfident, >1 = conservative.
    params.ReportedCovScale = 1.0;

    % --- bearing covariance regularisation ---
    % A unit bearing has only 2 true degrees of freedom, so the 3x3
    % ambient covariance built by GENERATEDETECTIONS is rank <= 2. A tiny
    % variance along the bearing direction itself keeps it positive
    % definite so it can be used with GaussianInfo.fromMoment.
    params.BearingRadialVariance = 1e-8;

    % --- ball ---
    params.Ball.FrictionCoeff     = 0.6;   % 1/s, dv/dt = -FrictionCoeff*v while rolling
    params.Ball.ChipKickEnabled   = false;
    params.Ball.Gravity           = 9.81;
    params.Ball.BounceRestitution = 0.5;
    params.Ball.BoundaryMode      = 'reflect'; % 'reflect' | 'clamp'
    params.Ball.InitialPosition   = [0, 0];

    params.Ball.Kicks = struct( ...
        't',        {3,           9,            15,           21}, ...
        'Velocity', {[2.5 1.0 0], [-2.0 1.5 0], [1.5 -2.5 0], [-1.0 -1.0 0]});

    % Example chip kick (only applied if params.Ball.ChipKickEnabled is true)
    params.Ball.ChipKicks = struct('t', {12}, 'Velocity', {[1.0, 0.5, 3.0]});

end
