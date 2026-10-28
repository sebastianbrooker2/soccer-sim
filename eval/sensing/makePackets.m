function packets = makePackets(gt, params)
%MAKEPACKETS Detection packets (format of run_5v5_decentralised.m) from ground truth.
%   PACKETS = MAKEPACKETS(GT, PARAMS). GT: t (Nt x1), ball (Nt x2),
%   pos (Nt x2 x6), theta (Nt x6), team (1x6). Observers are ids 1..3
%   (gt.team==1; uses find(gt.team==1,3)). Each scans at params rate with a
%   sinusoidal gaze sweep about its heading, detects the ball and the
%   other 5 robots via COMPUTEBALLDETECTION (TRUE pose) and draws
%   SIMULATERANGEBEARING. The packet carries the BELIEVED pose = truth +
%   AR(1) noise (std observerPosStd, corr. time posCorrTime) with that
%   covariance, and ObserverHeadingVariance = headingStd^2 (heading noise
%   is folded into bearing noise, as in the original model).
%   params.sensor.model: 'cone' (this legacy wedge+confidence model, default) or
%   'camera' (physical K1 head-camera projection, see MAKEPACKETSCAMERA).
%   Labels (diagnostic only): 'ball', 'team<j>' (gt id j<=3), 'opp<o>' (gt id 3+o).
%   Teammates are class 'robot' like opponents unless
%   params.dropTeammateDetections (default true), in which case they are not
%   emitted at all (their identity/pose is known over the network). Packets sorted by time.
%
%   Params (flat or one sub-struct deep; defaults if absent):
%     fov [rad] 120deg | range [m] 10 | rate [Hz] 3 | detNoiseStd [rad] 1deg
%     rangeNoiseStd [m] 0.3 | gazeAmplitude [rad] 80deg | gazePeriod [s] 6
%     observerPosStd [m] 0.1 | headingStd [rad] 2deg | posCorrTime [s] 5
%     (aliases: fovRad, rangeMax, sensorRate, bearingNoiseStd, ...)
%   Scalars or per-robot vectors (1x3) are accepted.

P = @(n, d) evalParamOr(params, n, d);
if strcmpi(P({'sensorModel', 'model'}, 'cone'), 'yolo')
    src = P({'yoloSource'}, '');
    if isempty(src), error('makePackets:yolo', 'sensor.model=yolo needs params.sensor.yoloSource (dets dir)'); end
    packets = makePacketsYolo(gt, params, src);
    return
end
if strcmpi(P({'sensorModel', 'model'}, 'cone'), 'camera')
    packets = makePacketsCamera(gt, params);
    return
end
fov    = P({'fov', 'fovRad', 'FOV'}, deg2rad(120));
rng_   = P({'range', 'rangeMax', 'Range', 'sensorRange'}, 10);
rate   = P({'rate', 'sensorRate', 'Rate'}, 3);
detStd = P({'detNoiseStd', 'bearingNoiseStd', 'DetectionNoiseStd', 'detectionNoiseStd'}, deg2rad(1));
rngStd = P({'rangeNoiseStd', 'RangeNoiseStd'}, 0.3);
gAmp   = P({'gazeAmplitude', 'GazeAmplitude'}, deg2rad(80));
gOmega = P({'GazeAngularSpeed', 'gazeAngularSpeed'}, 2*pi/P({'gazePeriod'}, 6));
gPhase = P({'GazePhase0', 'gazePhase0'}, 2*pi*(0:2)/3);
dropTm = P({'dropTeammateDetections'}, true);
posStd = P({'observerPosStd', 'selfPosStd'}, 0.1);
hdgStd = P({'headingStd', 'observerHeadingStd'}, deg2rad(2));
tau    = P({'posCorrTime'}, 5);
minBall  = P({'minRangeBall'}, 0);
minRobot = P({'minRangeRobot'}, 0);
at = @(v, r) v(min(r, numel(v)));

obs = find(gt.team == 1, 3, 'first');
if isempty(obs), obs = 1:3; end
nR = size(gt.pos, 3);
Nt = numel(gt.t);
t0 = gt.t(1); dtg = gt.t(2) - gt.t(1);
idx = @(t) min(max(round((t - t0)/dtg) + 1, 1), Nt);

packets = struct('time', {}, 'RobotID', {}, 'ObserverPosition', {}, 'ObserverCovariance', {}, ...
    'DetectionNoiseStd', {}, 'RangeNoiseStd', {}, 'ObserverHeadingVariance', {}, ...
    'LookHeading', {}, 'FOV', {}, 'Range', {}, 'detections', {});

for ri = 1:numel(obs)
    r = obs(ri);
    rate_r = at(rate, ri);
    ts = t0 + 1/rate_r : 1/rate_r : gt.t(end);
    phase0 = at(gPhase, ri);
    omega = at(gOmega, ri);
    pStd = at(posStd, ri);
    bias = pStd * randn(2, 1);
    a = exp(-(1/rate_r)/tau);
    C = pStd^2 * eye(2);
    for t = ts
        kk = idx(t);
        bias = a*bias + sqrt(1 - a^2)*pStd*randn(2, 1);
        truePos = squeeze(gt.pos(kk, :, r)).';
        heading = gt.theta(kk, r);
        gaze = at(gAmp, ri) * sin(phase0 + omega*t);

        sensor.Position   = truePos.';
        sensor.Heading    = heading;
        sensor.GazeOffset = gaze;
        sensor.FOV        = at(fov, ri);
        sensor.Range      = at(rng_, ri);

        pkt.time                    = t;
        pkt.RobotID                 = ri;
        pkt.ObserverPosition        = truePos + bias;
        pkt.ObserverCovariance      = C;
        pkt.DetectionNoiseStd       = at(detStd, ri);
        pkt.RangeNoiseStd           = at(rngStd, ri);
        pkt.ObserverHeadingVariance = at(hdgStd, ri)^2;
        pkt.LookHeading             = heading + gaze;   % believed heading ~ true (error enters via HeadingVariance)
        pkt.FOV                     = at(fov, ri);
        pkt.Range                   = at(rng_, ri);
        pkt.detections = struct('y', {}, 'Class', {}, 'TrueLabel', {}, 'LastDetection', {});

        % candidates: ball + the other robots
        cp = squeeze(gt.pos(kk, :, :));          % 2 x nR
        cls = {'ball'}; lab = {'ball'}; pos = {gt.ball(kk, :).'};
        for j = 1:nR
            if j == r, continue, end
            if dropTm && j <= 3, continue, end   % teammate (known via network)
            cls{end+1} = 'robot'; %#ok<AGROW>
            if j <= 3, lab{end+1} = sprintf('team%d', j); else, lab{end+1} = sprintf('opp%d', j - 3); end %#ok<AGROW>
            pos{end+1} = cp(:, j); %#ok<AGROW>
        end
        for c = 1:numel(cls)
            obj_.Position = pos{c}.';
            det = computeBallDetection(sensor, obj_);
            if ~det.Detected, continue, end
            if (strcmp(cls{c}, 'ball') && det.Distance < minBall) || ...
               (strcmp(cls{c}, 'robot') && det.Distance < minRobot), continue, end
            y = simulateRangeBearing(pkt, pos{c}, truePos);
            pkt.detections(end + 1) = struct('y', y, 'Class', cls{c}, ...
                'TrueLabel', lab{c}, 'LastDetection', det); %#ok<AGROW>
        end
        packets(end + 1) = pkt; %#ok<AGROW>
    end
end
[~, order] = sort([packets.time]);
packets = packets(order);
end
