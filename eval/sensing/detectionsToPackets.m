function packets = detectionsToPackets(det, cam, poses, params)
%DETECTIONSTOPACKETS Camera detections (bbox in pixels) -> tracker packets.
%   PACKETS = DETECTIONSTOPACKETS(DET, CAM, POSES, PARAMS) converts logged
%   YOLO / simulator-detector bounding boxes into the packet format produced
%   by MAKEPACKETS (time, RobotID, ObserverPosition, ObserverCovariance,
%   DetectionNoiseStd, RangeNoiseStd, ObserverHeadingVariance, LookHeading,
%   FOV, Range, detections{y=[ux;uy;r], Class, TrueLabel, LastDetection}).
%
%   DET     table, struct array, or CSV path with columns
%             t        [s]  detection (image) time
%             robot    int  observing robot id (1..N, as RobotID in packets)
%             class    str  'ball' | 'robot' (also 'Ball','Person','Opponent',
%                           'Teammate','Robot'); 'none' = frame with no
%                           detections (emits an empty packet); anything
%                           else (Goalpost, L_cross, ...) is ignored
%             u_px,v_px  bbox CENTRE in pixels (u right, v down)
%             w_px,h_px  bbox size in pixels
%             conf     0..1 detector confidence
%           (optional extra columns ignored)
%   CAM     struct, scalar or one element per robot id (CAM(robot)):
%             .fx .fy .cx .cy [px], .width .height [px]
%             .pivot   [3x1] head pivot in the body frame (x fwd,y left,z up,
%                      origin at the trunk reference point), m
%             .offset  [3x1] camera position relative to the pivot, in the
%                      head frame (rotates with head yaw/pitch), m
%             .pitch   [rad] fixed mount pitch (+ = looks down), default 0
%             .yaw     [rad] fixed mount yaw, default 0
%             .trunkHeight [m] height of the body-frame origin above the
%                      ground (z of the pivot frame); REQUIRED for range.
%           See K1SIMCAMERA for the simulator's K1 head camera.
%   POSES   struct with column vectors t, robot, x, y, theta (believed body
%           pose in the field frame; theta=0 faces +x), optional headYaw,
%           headPitch [rad; + = pitch down, yaw left] and optional
%           posStd [m]. Linearly interpolated per robot (nearest at ends).
%           A function handle @(robot,t) -> struct(x,y,theta,headYaw,
%           headPitch) is also accepted.
%   PARAMS  optional struct (defaults in brackets):
%     ballRadius [0.107] m   robotHeight [0.95] m   maxRange [10] m
%     pixStd [2] px    pitchStd [1 deg] rad   headStd [0.5 deg] rad
%     headingStd [2 deg] rad  posStd [0.1] m   minConf [0]
%     rangeMethod ['fuse'] 'ground' | 'size' | 'fuse'
%     keepClipped [false]   bbox cut by image border => unreliable range
%     rangeNoiseStd []   override the packet range std (else max over dets)
%     tOffset [0]        added to det.t (clock alignment)
%     model []           struct from YOLOERRORMODEL: replaces pixStd/sizeStd by the
%     calibrated, size-dependent box-error model (box-size bias
%     correction, per-detection range / bearing std with a
%     heavy-tail inflation, LowQuality flag; see locateModel)
%
%   Geometry. Pixel (u,v) -> optical ray (xn,yn,1), xn=(u-cx)/fx,
%   yn=(v-cy)/fy (x right, y down, z forward). Body-frame direction with
%   total pitch p (down +): d = f + xn*r + yn*dn, f=[cos p;0;-sin p],
%   r=[0;-1;0], dn=[-sin p;0;-cos p]; then rotated by total yaw (head+mount)
%   and robot heading. Camera centre = pivot + Rz(headYaw)Ry(pitch)*offset.
%   Range: GROUND = ray intersection with plane z=R (ball centre, bbox
%   centre) or z=0 (robot foot point = bbox bottom centre); SIZE = optical
%   depth Z = fx*2R/w (ball) or fy*Hrobot/h (robot). FUSE = inverse-variance
%   mean of the two (variances by finite differences of pixStd/pitchStd).
%   y = [u;r] uses the horizontal vector from the OBSERVER BODY position
%   (not the camera) to the target, as SIMULATERANGEBEARING does.
%   The detection struct also carries RangeStd (per-detection sigma) and
%   Conf; the packet's RangeNoiseStd is the max RangeStd (conservative).

if nargin < 4 || isempty(params), params = struct(); end
P = @(n, d) getOr(params, n, d);
ballR   = P('ballRadius', 0.107);
robH    = P('robotHeight', 0.95);
maxRng  = P('maxRange', 10);
pixStd  = P('pixStd', 2);
pitStd  = P('pitchStd', deg2rad(1));
hdStd   = P('headStd', deg2rad(0.5));
hdgStd  = P('headingStd', deg2rad(2));
posStd  = P('posStd', 0.1);
minConf = P('minConf', 0);
method  = P('rangeMethod', 'fuse');
keepCl  = P('keepClipped', false);
rngOvr  = P('rangeNoiseStd', []);
tOff    = P('tOffset', 0);
sizeStd = P('sizeStd', pixStd);      % px, bbox size std (range-from-size)
rngInfl = P('rangeInflate', 1);      % multiplier on per-detection range std
model   = P('model', []);            % calibrated YOLO error model (yoloErrorModel); [] = pixStd/sizeStd first-order

det = normaliseDet(det);
det.t = det.t + tOff;

emptyDet = struct('y', {}, 'Class', {}, 'TrueLabel', {}, 'LastDetection', {});
packets = struct('time', {}, 'RobotID', {}, 'ObserverPosition', {}, 'ObserverCovariance', {}, ...
    'DetectionNoiseStd', {}, 'RangeNoiseStd', {}, 'ObserverHeadingVariance', {}, ...
    'LookHeading', {}, 'FOV', {}, 'Range', {}, 'detections', {});

% group rows by (robot, time)
[keys, ~, g] = unique([det.robot(:), det.t(:)], 'rows');
for k = 1:size(keys, 1)
    ri = keys(k, 1); t = keys(k, 2);
    rows = find(g == k);
    c = cam(min(ri, numel(cam)));
    ps = poseAt(poses, ri, t);
    hy = getOr(ps, 'headYaw', 0); hp = getOr(ps, 'headPitch', 0);
    ps_std = getOr(ps, 'posStd', posStd);

    pkt.time = t;
    pkt.RobotID = ri;
    pkt.ObserverPosition = [ps.x; ps.y];
    pkt.ObserverCovariance = ps_std^2 * eye(2);
    pkt.DetectionNoiseStd = hypot(pixStd/c.fx, hdStd);
    pkt.RangeNoiseStd = 0;
    pkt.ObserverHeadingVariance = hdgStd^2;
    pkt.LookHeading = ps.theta + hy + getOr(c, 'yaw', 0);
    pkt.FOV = 2*atan(c.width/2/c.fx);
    pkt.Range = maxRng;
    pkt.detections = emptyDet;

    hasDet = false;
    for rr = rows(:).'
        cls = classOf(det.class{rr});
        if isempty(cls) || det.conf(rr) < minConf, continue, end
        if isempty(model)
            [pw, sig, ok, info] = locate(c, ps, hy, hp, cls, det, rr, ballR, robH, ...
                pixStd, pitStd, method, keepCl, sizeStd);
        else
            [pw, sig, ok, info] = locateModel(c, ps, hy, hp, cls, det, rr, ballR, robH, ...
                pitStd, method, keepCl, model);
        end
        if ~ok, continue, end
        d = pw - [ps.x; ps.y];
        r = norm(d);
        if r < 1e-6 || r > maxRng, continue, end
        u = d / r;
        lastDet = struct('Detected', true, 'Conf', det.conf(rr), 'Pixel', ...
            [det.u_px(rr) det.v_px(rr) det.w_px(rr) det.h_px(rr)], ...
            'Method', info.method, 'Range', r);
        pkt.detections(end+1) = struct('y', [u; r], 'Class', cls, ...
            'TrueLabel', det.class{rr}, 'LastDetection', lastDet); %#ok<AGROW>
        sig = sig * rngInfl;
        pkt.detections(end).LastDetection.RangeStd = sig;
        pkt.RangeNoiseStd = max(pkt.RangeNoiseStd, sig);
        if ~isempty(model)
            pkt.detections(end).LastDetection.LowQuality = info.lowQuality;
            pkt.detections(end).LastDetection.BearingStd = info.bearingStd;
            bsd = hypot(info.bearingStd, hdStd);
            if ~hasDet, pkt.DetectionNoiseStd = bsd; else, pkt.DetectionNoiseStd = max(pkt.DetectionNoiseStd, bsd); end
            hasDet = true;
        end
    end
    if ~isempty(rngOvr), pkt.RangeNoiseStd = rngOvr;
    elseif pkt.RangeNoiseStd == 0, pkt.RangeNoiseStd = 0.3; end   % no dets: nominal
    packets(end+1) = pkt; %#ok<AGROW>
end
if ~isempty(packets)
    [~, order] = sort([packets.time]);
    packets = packets(order);
end
end

% ------------------------------------------------------------------------
function [pw, sig, ok, info] = locate(c, ps, hy, hp, cls, det, rr, ballR, robH, pixStd, pitStd, method, keepCl, sizeStd)
u = det.u_px(rr); v = det.v_px(rr); w = det.w_px(rr); h = det.h_px(rr);
isBall = strcmp(cls, 'ball');
ok = false; pw = [NaN; NaN]; sig = NaN; info = struct('method', '');
if isBall
    vq = v; zPlane = ballR; S = 2*ballR; s = w; f = c.fx;
    clipped = (u - w/2 <= 0.5) || (u + w/2 >= c.width - 0.5);
else
    vq = v + h/2; zPlane = 0; S = robH; s = h; f = c.fy;     % foot point
    clipped = (v + h/2 >= c.height - 0.5) || (v - h/2 <= 0.5) ...
        || (u - w/2 <= 0.5) || (u + w/2 >= c.width - 0.5);
end
if clipped && ~keepCl, return, end

camPos = cameraPosition(c, ps, hy, hp);
rg = @(dv) groundPoint(c, ps, hy, hp, u, vq + dv, zPlane, camPos);

[pg, okg] = rg(0);
% size-based
Z = f * S / s;
dirb = rayWorld(c, ps, hy, hp, u, v);
ps_ = camPos + (Z / opticalDot(c, ps, hy, hp, dirb)) * dirb;
pz = ps_(1:2);

% variances (finite differences)
sigPix = hypot(pixStd, c.fy*pitStd);          % vertical px error incl. pitch error
if okg
    [pg2, ok2] = rg(sigPix);
    sg = norm(pg2 - pg); if ~ok2, sg = inf; end
    sg = max(sg, 1e-3);
else
    sg = inf;
end
rz = norm(pz - camPos(1:2));
sz = max(rz * hypot(sizeStd/s, 0.05), 1e-3);   % 5% metric-size uncertainty

switch method
    case 'ground'
        if okg, pw = pg; sig = sg; info.method = 'ground'; else, return, end
    case 'size'
        pw = pz; sig = sz; info.method = 'size';
    otherwise  % fuse
        if okg && isfinite(sg) && ~clipped
            wg = 1/sg^2; wz = 1/sz^2;
            pw = (wg*pg + wz*pz)/(wg + wz); sig = sqrt(1/(wg + wz)); info.method = 'fuse';
        elseif okg && ~isBall && ~clipped
            pw = pg; sig = sg; info.method = 'ground';
        elseif okg && isBall
            pw = pg; sig = sg; info.method = 'ground';
        elseif isBall || ~clipped
            pw = pz; sig = sz; info.method = 'size';
        else
            return
        end
end
ok = all(isfinite(pw));
end

% ------------------------------------------------------------------------
function [pw, sig, ok, info] = locateModel(c, ps, hy, hp, cls, det, rr, ballR, robH, pitStd, method, keepCl, model)
% Calibrated box-error model (see YOLOERRORMODEL). Measured box size s (ball
% width / robot height) -> unbiased size sHat and its std; ground row bias
% removed; first-order range std from the calibrated pixel stds, then a
% heavy-tail inflation k(range, conf, quality) fitted on real01.
u = det.u_px(rr); v = det.v_px(rr); w = det.w_px(rr); h = det.h_px(rr); cf = det.conf(rr);
isBall = strcmp(cls, 'ball');
ok = false; pw = [NaN; NaN]; sig = NaN;
info = struct('method', '', 'lowQuality', false, 'bearingStd', NaN);
if isBall
    M = model.ball; S = 2*ballR; s = w; f = c.fx; zPlane = ballR;
    clipped = (u - w/2 <= 0.5) || (u + w/2 >= c.width - 0.5);
else
    M = model.robot; S = robH; s = h; f = c.fy; zPlane = 0;
    clipped = (v + h/2 >= c.height - 0.5) || (v - h/2 <= 0.5) ...
        || (u - w/2 <= 0.5) || (u + w/2 >= c.width - 0.5);
end
if clipped && ~keepCl, return, end
ratio = interp1(M.s, M.sTrue./M.s, min(max(s, M.s(1)), M.s(end)));
rel   = interp1(M.s, M.sSd./M.sTrue, min(max(s, M.s(1)), M.s(end)));
sHat  = max(s*ratio, 0.5*s);
sdS   = sHat*rel;                                  % px std of the unbiased size
vB    = interp1(M.s, M.vBias, min(max(s, M.s(1)), M.s(end)));
vSd   = interp1(M.s, M.vSd,   min(max(s, M.s(1)), M.s(end)));
uSd   = interp1(M.s, M.uSd,   min(max(s, M.s(1)), M.s(end)));
cfac  = interpStep(M.cEdge, M.cFac, cf);
if isBall, vq = v - vB; else, vq = v + h/2 - vB; end
camPos = cameraPosition(c, ps, hy, hp);
rg = @(dv) groundPoint(c, ps, hy, hp, u, vq + dv, zPlane, camPos);
[pg, okg] = rg(0);
Z = f*S/sHat;
dirb = rayWorld(c, ps, hy, hp, u, v);
pz = camPos(1:2) + (Z / opticalDot(c, ps, hy, hp, dirb)) * dirb(1:2);
sigPix = hypot(vSd, c.fy*pitStd);
sg = inf;
if okg
    [pg2, ok2] = rg(sigPix); sg = norm(pg2 - pg); if ~ok2, sg = inf; end
    sg = max(sg, 1e-3);
end
rz = norm(pz - camPos(1:2));
sz = max(rz*sdS/sHat, 0.03*rz);
switch method
    case 'ground', if okg, pw = pg; sig = sg; info.method = 'ground'; else, return, end
    case 'size',   pw = pz; sig = sz; info.method = 'size';
    otherwise
        if okg && isfinite(sg)
            wg = 1/sg^2; wz = 1/sz^2;
            pw = (wg*pg + wz*pz)/(wg + wz); sig = sqrt(1/(wg + wz)); info.method = 'fuse';
        elseif okg, pw = pg; sig = sg; info.method = 'ground';
        else, pw = pz; sig = sz; info.method = 'size';
        end
end
if ~all(isfinite(pw)), return, end
rEst = norm(pw - camPos(1:2));
% quality
lq = false;
if isBall
    asp = abs(log(w/max(h, eps)));
    lq = (w < M.minW) || (asp > M.maxAspect) || (cf < M.confSmall && w < 1.25*M.minW);
else
    lq = h < M.minH;
end
kr = interp1(M.kR, M.kVal, min(max(rEst, M.kR(1)), M.kR(end)));
if lq && isfield(M, 'lqFac'), kr = kr*M.lqFac; end
sig = max(sig*kr*cfac, 0.05 + 0.02*rEst);
info.lowQuality = lq;
info.bearingStd = uSd*cfac/c.fx * interpOr(M, 'kB', 1);
ok = true;
end

function v = interpStep(edges, vals, x)
i = find(x >= edges(1:end-1), 1, 'last'); if isempty(i), i = 1; end
v = vals(min(i, numel(vals)));
end
function v = interpOr(M, f, d)
if isfield(M, f) && ~isempty(M.(f)), v = M.(f); else, v = d; end
end

function [p, ok] = groundPoint(c, ps, hy, hp, u, v, zPlane, camPos)
d = rayWorld(c, ps, hy, hp, u, v);
camZ = camPos(3);
if d(3) >= -1e-6 || camZ <= zPlane
    p = [NaN; NaN]; ok = false; return
end
lam = (zPlane - camZ) / d(3);
q = camPos + lam*d;
p = q(1:2); ok = true;
end

function d = rayWorld(c, ps, hy, hp, u, v)
xn = (u - c.cx)/c.fx; yn = (v - c.cy)/c.fy;
p = getOr(c, 'pitch', 0) + hp;
yaw = getOr(c, 'yaw', 0) + hy + ps.theta;
f = [cos(p); 0; -sin(p)]; r = [0; -1; 0]; dn = [-sin(p); 0; -cos(p)];
db = f + xn*r + yn*dn;
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
% db is in the (body rotated by head+mount yaw) frame; Rz applies all yaw
d = Rz*db;
d = d / norm(d);
end

function z = opticalDot(c, ps, hy, hp, dirw)
% cosine between ray and optical axis (world): Z-depth = lam * (d . axis)
p = getOr(c, 'pitch', 0) + hp;
yaw = getOr(c, 'yaw', 0) + hy + ps.theta;
ax = [cos(yaw)*cos(p); sin(yaw)*cos(p); -sin(p)];
z = dirw.' * ax;
end

function camPos = cameraPosition(c, ps, hy, hp)
p = hp;                                   % head pitch moves the camera on its arm
off = c.offset(:);
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
yb = hy; Rzh = [cos(yb) -sin(yb) 0; sin(yb) cos(yb) 0; 0 0 1];
pb = c.pivot(:) + Rzh*Ry*off;             % body frame
Rb = [cos(ps.theta) -sin(ps.theta) 0; sin(ps.theta) cos(ps.theta) 0; 0 0 1];
camPos = [ps.x + [1 0 0]*Rb*pb; ps.y + [0 1 0]*Rb*pb; c.trunkHeight + pb(3)];
end

function cls = classOf(s)
s = lower(strtrim(char(s)));
switch s
    case 'ball', cls = 'ball';
    case {'person', 'opponent', 'robot', 'teammate', 'player'}, cls = 'robot';
    otherwise, cls = '';
end
end

function ps = poseAt(poses, ri, t)
if isa(poses, 'function_handle'), ps = poses(ri, t); return, end
m = poses.robot(:) == ri;
tt = poses.t(m); tt = tt(:);
[tt, o] = sort(tt);
ps.x = interp1(tt, sub(poses.x, m, o), t, 'linear', 'extrap');
ps.y = interp1(tt, sub(poses.y, m, o), t, 'linear', 'extrap');
th = unwrap(sub(poses.theta, m, o));
ps.theta = interp1(tt, th, t, 'linear', 'extrap');
for fn = {'headYaw', 'headPitch'}
    if isfield(poses, fn{1})
        ps.(fn{1}) = interp1(tt, sub(poses.(fn{1}), m, o), t, 'linear', 'extrap');
    end
end
if isfield(poses, 'posStd'), ps.posStd = mean(sub(poses.posStd, m, o)); end
end

function v = sub(x, m, o)
x = x(:); x = x(m); v = x(o);
end

function det = normaliseDet(det)
if ischar(det) || isstring(det)
    det = readtable(char(det), 'TextType', 'char');
end
if istable(det)
    s = struct();
    for n = det.Properties.VariableNames, s.(n{1}) = det.(n{1}); end
    det = s;
elseif isstruct(det) && numel(det) > 1
    s = struct();
    for n = fieldnames(det).', s.(n{1}) = {det.(n{1})}; end
    det = s;
end
n = numel(det.t);
det.t = det.t(:); det.robot = det.robot(:);
if ~iscell(det.class), det.class = cellstr(det.class); end
if isstruct(det) && iscell(det.t), det.t = cell2mat(det.t(:)); end
for f = {'u_px', 'v_px', 'w_px', 'h_px', 'conf'}
    if ~isfield(det, f{1}), det.(f{1}) = nan(n, 1); end
    det.(f{1}) = det.(f{1})(:);
end
det.conf(isnan(det.conf)) = 1;
% 'none' rows: keep as empty-frame markers (classOf returns '' -> ignored)
end

function v = getOr(s, n, d)
if isstruct(s) && isfield(s, n) && ~isempty(s.(n)), v = s.(n); else, v = d; end
end
