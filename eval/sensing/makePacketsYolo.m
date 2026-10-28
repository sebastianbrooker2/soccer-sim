function [packets, info] = makePacketsYolo(gt, params, detsSrc)
%MAKEPACKETSYOLO Packets from logged YOLO detections (real detector output).
%   [PACKETS, INFO] = MAKEPACKETSYOLO(GT, PARAMS, DETSSRC)
%   Same packet format as MAKEPACKETSCAMERA, but the pixel boxes come from
%   a detector log instead of an ideal projection of ground truth.
%   DETSSRC: a directory holding dets.csv (+ dets_frames.csv), e.g.
%   recordings/real01/yolo, or a struct with fields .dets and .frames (CSV
%   paths or tables). Schemas:
%     dets.csv        t,robot,class,u_px,v_px,w_px,h_px,conf
%                     class: Ball|Robot|Goalpost|L_cross|T_cross|X_cross
%                     (goalposts / intersections are ignored); pixels in the
%                     320x240 image; t on the gt.t time base; robot 1..3
%                     (0-based ids are shifted if a 0 occurs).
%     dets_frames.csv t,robot,nDets   one row per processed frame, incl.
%                     empty frames. Every frame emits one packet (empty when
%                     nothing survives), so the tracker can count misses via
%                     EXPECTEDVISIBLE (FOV / Range / LookHeading fields).
%   Observer pose = BELIEVED pose = truth + AR(1) bias (as MAKEPACKETSCAMERA),
%   head yaw / pitch / trunk tilt from gt (+ headStd / tiltStd noise).
%   Boxes -> [bearing; range] by DETECTIONSTOPACKETS (ball: size + ground
%   plane fusion, robot: foot point + height).
%   params.sensor.yolo (all optional, see evalParams):
%     minConf struct/scalar  per-class threshold: .ball .robot
%     pixStd  [px] bbox centre std     sizeStd [px] bbox size std
%     rangeInflate  multiplier on the per-detection range std
%     maxRange [m] conversion cutoff   expectRange [m] packet.Range (used
%     for expected-visible miss counting; FOV = camera HFOV)
%     teammateGate [px] extra gate around the projected teammate box
%   params.dropTeammateDetections (default true): a Robot box whose centre
%     lies within the (gate-enlarged) projected box of a teammate GT
%     position is discarded. This models teammate identity from team comms
%     (teammate poses are known over the network); it uses GT teammate
%     positions as a stand-in for the communicated ones and the believed
%     own pose. An opponent standing in front of / behind a teammate can be
%     dropped too. False positives and opponent detections pass untouched.
%   detections(k).TrueLabel ('ball','teamJ','oppJ','fp') is set by projecting
%   GT for diagnostics ONLY (never used by trackers).
%   INFO per observer: frames, ball/robot detections kept, dropped counts.
P = @(n, d) evalParamOr(params, n, d);
Y = P({'yolo'}, struct());
yv = @(n, d) fieldOr(Y, n, d);
dropTm  = P({'dropTeammateDetections'}, true);
posStd  = P({'observerPosStd', 'selfPosStd'}, 0.1);
hdgStd  = P({'headingStd', 'observerHeadingStd'}, deg2rad(2));
tau     = P({'posCorrTime'}, 5);
headStd = P({'headStd'}, deg2rad(0.5));
tiltStd = P({'tiltStd'}, deg2rad(0.5));
pitStd  = P({'pitchStd'}, deg2rad(1));
fbPitch = P({'fallbackHeadPitch'}, 0.25);
trunkH  = P({'trunkHeight'}, 0.55);
ballR   = P({'ballRadius'}, 0.107);
robH    = P({'robotHeight'}, 0.95);
robW    = P({'robotWidth'}, 0.30);
pixStd  = yv('pixStd', 2);
sizeStd = yv('sizeStd', pixStd);
mc = yv('minConf', 0.3);
if isstruct(mc), mcB = fieldOr(mc, 'ball', 0.3); mcR = fieldOr(mc, 'robot', 0.3);
else, mcB = mc; mcR = mc; end
maxRng  = yv('maxRange', 20);
expRng  = yv('expectRange', 8);
gate    = yv('teammateGate', 10);

[D, F] = loadDets(detsSrc);
obs = find(gt.team == 1, 3, 'first'); if isempty(obs), obs = 1:3; end
nR = size(gt.pos, 3); Nt = numel(gt.t);
t0 = gt.t(1); dtg = gt.t(2) - gt.t(1);
idx = @(t) min(max(round((t - t0)/dtg) + 1, 1), Nt);
hasHead = isfield(gt, 'hasHead') && ~isempty(gt.hasHead) && gt.hasHead ...
    && isfield(gt, 'headYaw') && isfield(gt, 'headPitch');
hasZ = hasHead && isfield(gt, 'z') && isfield(gt, 'roll') && isfield(gt, 'pitch');
if hasZ, trunkH = median(gt.z(:, obs), 'all', 'omitnan'); end
cam = k1SimCamera(trunkH);
opt = struct('ballRadius', ballR, 'robotHeight', robH, 'robotWidth', robW, ...
    'znear', 0.05, 'zfar', 30, 'minVisible', 0.0, 'minBallPx', 0, 'minRobotPx', 0);
convParams = struct('ballRadius', ballR, 'robotHeight', robH, 'maxRange', maxRng, ...
    'pixStd', pixStd, 'sizeStd', sizeStd, 'rangeInflate', yv('rangeInflate', 1), ...
    'pitchStd', pitStd, 'headStd', headStd, 'headingStd', yv('headingStd', hdgStd), 'posStd', posStd, ...
    'model', yv('model', []), 'rangeMethod', P({'rangeMethod'}, 'fuse'), 'keepClipped', P({'keepClipped'}, false));

packets = []; info = struct('observer', {}, 'frames', {}, 'ball', {}, 'robot', {}, ...
    'lowConf', {}, 'teammateDropped', {}, 'emitted', {}, 'falsePositive', {});
for ri = 1:numel(obs)
    r = obs(ri);
    Dm = D.robot == ri; Fm = F.robot == ri;
    ts = unique(round([F.t(Fm); D.t(Dm)], 4));
    n = numel(ts);
    if n == 0, continue, end
    pStd = posStd(min(ri, numel(posStd)));
    dtf = median(diff(ts)); if isempty(dtf) || ~isfinite(dtf) || dtf <= 0, dtf = 1/15; end
    a = exp(-dtf/tau);
    bias = pStd*randn(2, 1);
    poses = struct('t', ts, 'robot', repmat(ri, n, 1), 'x', zeros(n, 1), 'y', zeros(n, 1), ...
        'theta', zeros(n, 1), 'headYaw', zeros(n, 1), 'headPitch', zeros(n, 1), 'posStd', repmat(pStd, n, 1));
    trueCam = cell(n, 1);
    for k = 1:n
        t = ts(k); kk = idx(t);
        bias = a*bias + sqrt(1 - a^2)*pStd*randn(2, 1);
        th = gt.theta(kk, r); xy = squeeze(gt.pos(kk, :, r)).';
        if hasHead, hy = gt.headYaw(kk, r); hp = gt.headPitch(kk, r);
        else, hy = 0; hp = fbPitch; end
        if hasZ, z = gt.z(kk, r); rl = gt.roll(kk, r); pc = gt.pitch(kk, r);
        else, z = trunkH; rl = 0; pc = 0; end
        effPitch = hp + (pc*cos(hy) - rl*sin(hy));
        poses.x(k) = xy(1) + bias(1); poses.y(k) = xy(2) + bias(2); poses.theta(k) = th;
        poses.headYaw(k) = hy + headStd*randn;
        poses.headPitch(k) = effPitch + hypot(headStd, tiltStd)*randn;
        trueCam{k} = {[xy; z], th, rl, pc, hy, hp, [xy + bias; z]};
    end
    % detection rows (filtered)
    rows = struct('t', [], 'robot', [], 'class', {{}}, 'u_px', [], 'v_px', [], 'w_px', [], 'h_px', [], 'conf', []);
    di = find(Dm);
    nLow = 0; nTm = 0; nB = 0; nRb = 0;
    [~, fk] = ismember(round(D.t(di), 4), ts);
    for q = 1:numel(di)
        j = di(q); cl = lower(D.class{j});
        if strcmp(cl, 'ball'), if D.conf(j) < mcB, nLow = nLow + 1; continue, end
        elseif strcmp(cl, 'robot')
            if D.conf(j) < mcR, nLow = nLow + 1; continue, end
            if dropTm && isTeammate(gt, trueCam{fk(q)}, cam, D, j, r, obs, opt, gate, idx(ts(fk(q))))
                nTm = nTm + 1; continue
            end
        else, continue
        end
        if strcmp(cl, 'ball'), nB = nB + 1; else, nRb = nRb + 1; end
        rows.t(end+1, 1) = ts(fk(q)); rows.robot(end+1, 1) = ri; rows.class{end+1, 1} = cl; %#ok<AGROW>
        rows.u_px(end+1, 1) = D.u_px(j); rows.v_px(end+1, 1) = D.v_px(j);
        rows.w_px(end+1, 1) = D.w_px(j); rows.h_px(end+1, 1) = D.h_px(j); rows.conf(end+1, 1) = D.conf(j);
    end
    % frame markers: ensure every frame yields a packet
    have = ismember(ts, unique(rows.t));
    for k = find(~have).'
        rows.t(end+1, 1) = ts(k); rows.robot(end+1, 1) = ri; rows.class{end+1, 1} = 'none'; %#ok<AGROW>
        rows.u_px(end+1, 1) = 0; rows.v_px(end+1, 1) = 0; rows.w_px(end+1, 1) = 0;
        rows.h_px(end+1, 1) = 0; rows.conf(end+1, 1) = 1;
    end
    pk = detectionsToPackets(rows, cam, poses, convParams);
    nfp = 0;
    for q = 1:numel(pk)
        % additive: raw boxes (post conf/teammate filter, BEFORE the SRIF range conversion
        % dropped clipped / out-of-range ones) + believed camera pose, for NUbots-style
        % measurement (nubotsMeasurement). Packets of other trackers ignore these fields.
        kq0 = find(abs(ts - pk(q).time) < 1e-6, 1);
        pk(q).Cam = cam;
        pk(q).CamPose = struct('x', poses.x(kq0), 'y', poses.y(kq0), 'theta', poses.theta(kq0), ...
            'headYaw', poses.headYaw(kq0), 'headPitch', poses.headPitch(kq0), 'trunkHeight', trunkH);
        mr = find(abs(rows.t - pk(q).time) < 1e-6 & ~strcmp(rows.class, 'none'));
        rb = struct('Class', {}, 'Box', {}, 'TrueLabel', {});
        for jj = mr(:).'
            rb(end+1) = struct('Class', rows.class{jj}, ... %#ok<AGROW>
                'Box', [rows.u_px(jj) rows.v_px(jj) rows.w_px(jj) rows.h_px(jj) rows.conf(jj)], 'TrueLabel', '');
        end
        pk(q).RawBoxes = rb;
        pk(q).FOV = 2*atan(cam.width/2/cam.fx);
        pk(q).Range = expRng;
        kq = find(abs(ts - pk(q).time) < 1e-6, 1);
        tc = trueCam{kq}; kk = idx(pk(q).time);
        for d = 1:numel(pk(q).detections)
            ld = pk(q).detections(d).LastDetection;
            [lab, tgt, tbox] = trueLabel(gt, tc, cam, ld.Pixel, pk(q).detections(d).Class, r, nR, kk, opt, ballR);
            % diagnostics only: true horizontal offset from the believed observer position
            if tgt > 0, txy = squeeze(gt.pos(kk, :, tgt)).'; else, txy = gt.ball(kk, :).'; end
            pk(q).detections(d).TrueBox = tbox;
            if strcmp(lab, 'fp'), pk(q).detections(d).TrueDelta = [NaN; NaN];
            else, pk(q).detections(d).TrueDelta = txy(1:2) - pk(q).ObserverPosition(:); end
            if strcmp(lab, 'fp'), nfp = nfp + 1; end
            pk(q).detections(d).TrueLabel = lab;
        end
    end
    if isempty(packets), packets = pk; else, packets = [packets, pk]; end %#ok<AGROW>
    info(end+1) = struct('observer', ri, 'frames', n, 'ball', nB, 'robot', nRb, 'lowConf', nLow, ...
        'teammateDropped', nTm, 'emitted', sum(arrayfun(@(p) numel(p.detections), pk)), ...
        'falsePositive', nfp); %#ok<AGROW>
end
if ~isempty(packets)
    [~, order] = sort([packets.time]);
    packets = packets(order);
end
end

% ------------------------------------------------------------------------
function tf = isTeammate(gt, tc, cam, D, j, r, obs, opt, gate, kk)
% believed own pose camera, teammate positions as known over comms (GT)
tf = false;
[cp, Rc] = cameraPoseK1(cam, tc{7}, tc{2}, tc{3}, tc{4}, tc{5}, tc{6});
for m = obs(:).'
    if m == r, continue, end
    o = cameraProject('robot', [squeeze(gt.pos(kk, :, m)).'; 0], cp, Rc, cam, opt);
    if isnan(o.depth) || o.depth < opt.znear || o.depth > opt.zfar, continue, end
    b = o.full;   % unclipped box [uc vc w h]
    if abs(D.u_px(j) - b(1)) <= b(3)/2 + gate && abs(D.v_px(j) - b(2)) <= b(4)/2 + gate
        tf = true; return
    end
end
end

function [lab, tgt, tbox] = trueLabel(gt, tc, cam, pix, cls, r, nR, kk, opt, ballR)
% diagnostic: nearest GT projection to the box centre (true camera pose)
[cp, Rc] = cameraPoseK1(cam, tc{1}, tc{2}, tc{3}, tc{4}, tc{5}, tc{6});
best = inf; lab = 'fp'; tgt = 0; tbox = nan(1, 5);
if strcmp(cls, 'ball')
    o = cameraProject('ball', [gt.ball(kk, :).'; ballR], cp, Rc, cam, opt);
    if ~isnan(o.depth), best = norm(pix(1:2) - o.full(1:2)); 
        if best < max(8, o.full(3)); lab = 'ball'; tbox = [o.full o.depth]; end, end
    return
end
for j = 1:nR
    if j == r, continue, end
    o = cameraProject('robot', [squeeze(gt.pos(kk, :, j)).'; 0], cp, Rc, cam, opt);
    if isnan(o.depth) || o.depth < opt.znear, continue, end
    dd = norm(pix(1:2) - o.full(1:2));
    if dd < best && dd < max(10, 0.75*max(o.full(3:4)))
        best = dd; tgt = j; tbox = [o.full o.depth];
        if j <= 3, lab = sprintf('team%d', j); else, lab = sprintf('opp%d', j - 3); end
    end
end
end

function [D, F] = loadDets(src)
if ischar(src) || isstring(src)
    d = char(src);
    if isfolder(d), src = struct('dets', fullfile(d, 'dets.csv'), 'frames', fullfile(d, 'dets_frames.csv'));
    else, src = struct('dets', d, 'frames', ''); end
end
D = tbl(src.dets);
if isfield(src, 'frames') && ~isempty(src.frames) && (istable(src.frames) || isfile(src.frames))
    F = tbl(src.frames);
else
    F = struct('t', zeros(0, 1), 'robot', zeros(0, 1), 'nDets', zeros(0, 1));
end
if min([D.robot; F.robot; 1]) == 0, D.robot = D.robot + 1; F.robot = F.robot + 1; end
end

function s = tbl(x)
if ischar(x) || isstring(x)
    x = readtable(char(x), 'TextType', 'char');
end
if istable(x)
    s = struct();
    for n = x.Properties.VariableNames, s.(n{1}) = x.(n{1}); end
else
    s = x;
end
s.t = s.t(:); s.robot = s.robot(:);
if isfield(s, 'class')
    if ~iscell(s.class), s.class = cellstr(string(s.class)); end
    n = numel(s.t);
    for f = {'u_px', 'v_px', 'w_px', 'h_px', 'conf'}
        if ~isfield(s, f{1}), s.(f{1}) = ones(n, 1); end
        s.(f{1}) = s.(f{1})(:);
    end
    s.conf(isnan(s.conf)) = 1;
end
end

function v = fieldOr(s, n, d)
if isstruct(s) && isfield(s, n) && ~isempty(s.(n)), v = s.(n); else, v = d; end
end
