function [packets, info] = makePacketsCamera(gt, params)
%MAKEPACKETSCAMERA Packets from a physically projected K1 head camera.
%   Called by MAKEPACKETS when params.sensor.model == 'camera'.
%   For each observer (gt.team==1, first 3) and each frame time (rate
%   params.sensor.Rate, default 15 Hz; the sim detector runs <= 30 Hz):
%     1. camera pose from trunk pose (x,y,theta, + z/roll/pitch if gt.hasHead),
%        head yaw/pitch (gt.headYaw/headPitch if gt.hasHead, else the
%        sinusoidal gaze sweep + fixed fallbackHeadPitch), K1SIMCAMERA mount;
%     2. ball (sphere r=0.107, centre z=r) and the other robots (cylinder
%        0.95 x 0.30 m) are projected (CAMERAPROJECT, the sim detector's 50 %
%        visible rule); teammates dropped if params.dropTeammateDetections;
%     3. Gaussian pixel noise pixStd on bbox centre and size;
%     4. DETECTIONSTOPACKETS converts boxes to [u; range] (ground-plane /
%        size fused range) using the BELIEVED pose (truth + AR(1) bias), with
%        MEASURED head/tilt angles (truth + headStd / tiltStd noise); the
%        tilt enters as an effective extra head pitch pitch*cos(yaw)-roll*sin(yaw).
%   Per-detection range std is in detections(k).LastDetection.RangeStd.
%   Second output INFO: per-observer visible/emitted counts.
P = @(n, d) evalParamOr(params, n, d);
rate   = P({'rate', 'sensorRate', 'Rate'}, 15);
rngMax = P({'camZfar'}, 20);   % sim detector: ball detected out to >= 14.4 m (det01); 20 m ~ no limit on a 14x9 field
gAmp   = P({'gazeAmplitude', 'GazeAmplitude'}, deg2rad(45));
gOmega = P({'GazeAngularSpeed', 'gazeAngularSpeed'}, 2*pi/P({'gazePeriod'}, 6));
gPhase = P({'GazePhase0', 'gazePhase0'}, 2*pi*(0:2)/3);
dropTm = P({'dropTeammateDetections'}, true);
posStd = P({'observerPosStd', 'selfPosStd'}, 0.1);
hdgStd = P({'headingStd', 'observerHeadingStd'}, deg2rad(2));
tau    = P({'posCorrTime'}, 5);
pixStd = P({'pixStd'}, 1.5);
headStd = P({'headStd'}, deg2rad(0.5));
tiltStd = P({'tiltStd'}, deg2rad(0.5));
pitStd  = P({'pitchStd'}, deg2rad(1));
fbPitch = P({'fallbackHeadPitch'}, 0.25);
trunkH  = P({'trunkHeight'}, 0.55);
ballR   = P({'ballRadius'}, 0.107);
robH    = P({'robotHeight'}, 0.95);
robW    = P({'robotWidth'}, 0.30);
opt = struct('ballRadius', ballR, 'robotHeight', robH, 'robotWidth', robW, ...
    'znear', P({'znear'}, 0.05), 'zfar', rngMax, 'minVisible', 0.5, ...
    'minBallPx', P({'minBallPx'}, 0), 'minRobotPx', P({'minRobotPx'}, 8));
at = @(v, r) v(min(r, numel(v)));

obs = find(gt.team == 1, 3, 'first'); if isempty(obs), obs = 1:3; end
nR = size(gt.pos, 3); Nt = numel(gt.t);
t0 = gt.t(1); dtg = gt.t(2) - gt.t(1);
idx = @(t) min(max(round((t - t0)/dtg) + 1, 1), Nt);
hasHead = isfield(gt, 'hasHead') && ~isempty(gt.hasHead) && gt.hasHead ...
    && isfield(gt, 'headYaw') && isfield(gt, 'headPitch');
hasZ = hasHead && isfield(gt, 'z') && isfield(gt, 'roll') && isfield(gt, 'pitch');
if hasZ, trunkH = median(gt.z(:, obs), 'all', 'omitnan'); end
cam = k1SimCamera(trunkH);

convParams = struct('ballRadius', ballR, 'robotHeight', robH, 'maxRange', rngMax, ...
    'pixStd', pixStd, 'pitchStd', pitStd, 'headStd', headStd, 'headingStd', hdgStd, ...
    'posStd', posStd, 'rangeMethod', P({'rangeMethod'}, 'fuse'), ...
    'keepClipped', P({'keepClipped'}, false));

packets = []; info = struct('observer', {}, 'frames', {}, 'ballVisible', {}, 'robotVisible', {}, 'emitted', {});
for ri = 1:numel(obs)
    r = obs(ri);
    rate_r = at(rate, ri);
    ts = (t0 + 1/rate_r : 1/rate_r : gt.t(end)).';
    n = numel(ts);
    pStd = at(posStd, ri);
    a = exp(-(1/rate_r)/tau);
    bias = pStd*randn(2, 1);
    ph0 = at(gPhase, ri); om = at(gOmega, ri);
    poses = struct('t', ts, 'robot', repmat(ri, n, 1), 'x', zeros(n, 1), 'y', zeros(n, 1), ...
        'theta', zeros(n, 1), 'headYaw', zeros(n, 1), 'headPitch', zeros(n, 1), 'posStd', repmat(pStd, n, 1));
    rows = struct('t', [], 'robot', [], 'class', {{}}, 'u_px', [], 'v_px', [], 'w_px', [], 'h_px', [], 'conf', []);
    nBallVis = 0; nRobVis = 0;
    for k = 1:n
        t = ts(k); kk = idx(t);
        bias = a*bias + sqrt(1 - a^2)*pStd*randn(2, 1);
        th = gt.theta(kk, r);
        xy = squeeze(gt.pos(kk, :, r)).';
        if hasHead
            hy = gt.headYaw(kk, r); hp = gt.headPitch(kk, r);
        else
            hy = at(gAmp, ri)*sin(ph0 + om*t); hp = fbPitch;
        end
        if hasZ
            z = gt.z(kk, r); rl = gt.roll(kk, r); pc = gt.pitch(kk, r);
        else
            z = trunkH; rl = 0; pc = 0;
        end
        [cp, Rc] = cameraPoseK1(cam, [xy; z], th, rl, pc, hy, hp);
        % measured angles
        effPitch = hp + (pc*cos(hy) - rl*sin(hy));
        poses.x(k) = xy(1) + bias(1); poses.y(k) = xy(2) + bias(2);
        poses.theta(k) = th;
        poses.headYaw(k) = hy + headStd*randn;
        poses.headPitch(k) = effPitch + hypot(headStd, tiltStd)*randn;
        nd = 0;
        for c = 0:nR
            if c == 0
                kind = 'ball'; Pw = [gt.ball(kk, :).'; ballR]; code = 0;
            else
                j = c;
                if j == r || (dropTm && j <= 3), continue, end
                kind = 'robot'; Pw = [squeeze(gt.pos(kk, :, j)).'; 0]; code = j;
            end
            o = cameraProject(kind, Pw, cp, Rc, cam, opt);
            if ~o.visible, continue, end
            if c == 0, nBallVis = nBallVis + 1; else, nRobVis = nRobVis + 1; end
            b = o.bbox + pixStd*randn(1, 4);
            b(3:4) = max(b(3:4), 1);
            rows.t(end+1, 1) = t; rows.robot(end+1, 1) = ri; rows.class{end+1, 1} = kind; %#ok<AGROW>
            rows.u_px(end+1, 1) = b(1); rows.v_px(end+1, 1) = b(2);
            rows.w_px(end+1, 1) = b(3); rows.h_px(end+1, 1) = b(4);
            rows.conf(end+1, 1) = code; nd = nd + 1;
        end
        if nd == 0   % empty frame marker -> empty packet
            rows.t(end+1, 1) = t; rows.robot(end+1, 1) = ri; rows.class{end+1, 1} = 'none'; %#ok<AGROW>
            rows.u_px(end+1, 1) = 0; rows.v_px(end+1, 1) = 0; rows.w_px(end+1, 1) = 0;
            rows.h_px(end+1, 1) = 0; rows.conf(end+1, 1) = 1;
        end
    end
    pk = detectionsToPackets(rows, cam, poses, convParams);
    for q = 1:numel(pk)
        pk(q).FOV = 2*atan(cam.width/2/cam.fx);
        pk(q).Range = rngMax;
        for d = 1:numel(pk(q).detections)
            ld = pk(q).detections(d).LastDetection;
            j = round(ld.Conf);
            if strcmp(pk(q).detections(d).Class, 'ball'), lab = 'ball';
            elseif j <= 3, lab = sprintf('team%d', j); else, lab = sprintf('opp%d', j - 3); end
            pk(q).detections(d).TrueLabel = lab;
            ld.Conf = 1; pk(q).detections(d).LastDetection = ld;
        end
    end
    packets = [packets, pk]; %#ok<AGROW>
    info(end+1) = struct('observer', ri, 'frames', n, 'ballVisible', nBallVis, ...
        'robotVisible', nRobVis, 'emitted', sum(arrayfun(@(p) numel(p.detections), pk))); %#ok<AGROW>
end
[~, order] = sort([packets.time]);
packets = packets(order);
end
