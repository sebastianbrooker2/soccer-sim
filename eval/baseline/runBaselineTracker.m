function copies = runBaselineTracker(packets, params)
%RUNBASELINETRACKER Per-robot baseline tracker, 'faithful' or 'tuned' mode
%   (params.baseline.mode, see baselineDefaults).
%   copies(r).tracks(i): name, class, t (1xn), x (4xn), P (4x4xn), alive,
%   SpawnTime, deletedAt, misses, hold.   copies(r).b = parameters used.
%
%   Robot r uses ONLY packets with RobotID==r (plus, in faithful mode with
%   useTeamBall, the ball estimates teammates broadcast - as NUbots does).
%   TrueLabel is read ONLY for the teammate filter below.
%
%   TEAMMATE FILTER: when params.dropTeammateDetections is true (default),
%   detections whose TrueLabel starts with 'team' are ignored. This uses the
%   label as a STAND-IN for the teammate id that in reality comes from network
%   communication (RoboCup messages carry the sender pose). No other use of
%   TrueLabel is made.
%
%   FAITHFUL mode = NUbots_K1 behaviour (module/localisation):
%    robots (RobotLocalisation.cpp, cfg: R=0.1 I, Q pos 0.1/vel 0.01 *dt,
%      P0 1e-3, association_distance 0.5, max_missed_count 25, field+1 m):
%      * linear CV filter (UKF with linear model == KF), predicted every frame
%      * detections processed SEQUENTIALLY, each to the nearest track (state
%        already updated by earlier detections of the same frame; many-to-one
%        allowed, no gating beyond association_distance); farther -> new track
%      * missed_count++ only if the track's predicted position is in view
%        (stand-in for the green horizon: packet FOV and Range); out of view ->
%        treated as seen; deleted when missed_count > maxMisses
%      * deleted when outside field +- fieldMargin (params.field, default 14x9)
%      * proximity removal (default ON): NUbots MAIN RobotLocalisation.cpp:256-261, track dropped
%        if within association_distance of ANY already-kept robot (std::any_of). The NUbots_K1
%        branch (RobotLocalisation.cpp:256-270) has this commented out and a dead inner-loop
%        `continue`; reproduce that with b.k1ProximityBug = true.
%      * reference for ALL tracking logic = ~/NUbots (main); Ball/RobotLocalisation differ from
%        ~/NUbots_K1 only in the proximity rule (plus message plumbing: RoboCup vs Message,
%        teammate cost filter, ball echo test), none of which the baseline uses.
%      * measurement (faithful, b.measurement='nubots'): nubotsMeasurement from raw YOLO boxes
%    ball (BallLocalisation.cpp, R=0.1 I, Q 0.01*dt, init mean (5,0), cov 1e-3):
%      * ONE filter, never created/deleted; the nearest ball detection to the
%        STALE state is always fused (accept_ball is always true in the code)
%      * time() is called only when a measurement arrives, so the emitted state
%        is not predicted between measurements (track.hold = true)
%      * first dt = time since start (robot's first packet)
%      * low_confidence uses the SQUARED distance vs acceptance_radius(2), as in
%        the code; while low-confidence and a valid team guess exists, the team
%        guess is emitted instead and the filter is not updated
%      * team ball: each robot broadcasts its ball estimate every teamMsgPeriod
%        (delivered w.p. 1-linkDropProb); if no own update for
%        teamGuessDefaultTimer s, a received guess is emitted (average of guesses
%        heard within teamBallRecency s, no validity check, as in the code).
%        Emitted covariance = teamBallStd^2 I (NUbots emits none: assumption).
%      * the evaluated estimate is the most recently emitted ball.
%   TUNED mode: R from range/bearing/self-position noise, Mahalanobis chi2 99%
%    gating, global one-to-one greedy assignment, age-out after maxAge (3 s),
%    duplicates merged, field-boundary deletion, single ball track that is
%    re-initialised after ballReacquire consecutive gated-out ball detections.
if nargin < 2, params = struct(); end
b = baselineDefaults(params);
tuned = strcmp(b.mode, 'tuned');
dropTm = true;
if isfield(params, 'dropTeammateDetections') && ~isempty(params.dropTeammateDetections), dropTm = params.dropTeammateDetections; end
fl = 14; fw = 9;
if isfield(params, 'field') && isfield(params.field, 'length'), fl = params.field.length; fw = params.field.width; end
pDrop = 0; if isfield(params, 'linkDropProb'), pDrop = params.linkDropProb; end
if isempty(packets), copies = struct('robot', {}, 'tracks', {}); return; end
[~, o] = sort([packets.time]); packets = packets(o);
numObs = max([packets.RobotID]);
rs = RandStream('mt19937ar', 'Seed', 12345);

proto = emptyTrack(); proto(1) = [];
S = repmat(struct('act', proto, 'done', proto, 'counter', 0, 'ball', emptyTrack(), ...
    'bF', [], 'tLast', NaN, 'firstSeen', false, 'rej', 0, 'gTime', -inf(1, numObs), ...
    'gPos', zeros(2, numObs), 'nextMsg', 0, 'lastBallUpd', -inf, 'ballOn', false), 1, numObs);
for r = 1:numObs, S(r).nextMsg = (r-1)/numObs*b.teamMsgPeriod; end

for k = 1:numel(packets)
    p = packets(k); r = p.RobotID; t = p.time; s = S(r);
    if isnan(s.tLast), s.tLast = t - 1/15; end
    if ~tuned && strcmp(b.measurement, 'nubots') && isfield(p, 'RawBoxes') && isfield(p, 'CamPose') && ~isempty(p.CamPose)
        [pos, R, cls] = readNubots(p, b, dropTm);
    else
        [pos, R, cls] = readDetections(p, b, dropTm);
    end
    if tuned
        s = tunedStep(s, p, t, pos, R, cls, b, fl, fw);
    else
        s = faithfulRobots(s, p, t, pos, cls, b, fl, fw);
        s = faithfulBall(s, t, pos(:, strcmp(cls, 'ball')), b);
        % robocup message broadcast by this robot
        if b.useTeamBall && t >= s.nextMsg
            s.nextMsg = s.nextMsg + b.teamMsgPeriod*max(1, floor((t - s.nextMsg)/b.teamMsgPeriod) + 1);
            if ~isempty(s.bF) && t - s.lastBallUpd <= b.teamBallMaxAge
                S(r) = s; pk = s.bF.x(1:2);
                for q = [1:r-1, r+1:numObs]
                    if rand(rs) < pDrop, continue; end
                    S(q) = receiveGuess(S(q), r, t, pk, b);
                end
                s = S(r);
            end
        end
    end
    S(r) = s;
end

copies = repmat(struct('robot', 0, 'tracks', [], 'kind', 'baseline', 'b', b), 1, numObs);
for r = 1:numObs
    tr = [S(r).done, S(r).act];
    if S(r).ball.alive || ~isempty(S(r).ball.t), tr = [tr, S(r).ball]; end %#ok<AGROW>
    copies(r).robot = r; copies(r).tracks = tr;
end
end

%% ------------------------------------------------------------------ input
function [pos, R, cls] = readNubots(p, b, dropTm)
% NUbots_K1 measurement from the RAW YOLO boxes (nubotsMeasurement): ground-plane
% projection of the box centre (ball) / bottom-centre (robot) with the believed camera
% pose; no range/size cue, no per-detection covariance (R is the constant b.R).
rb = p.RawBoxes; nd = numel(rb);
pos = zeros(2, nd); cls = cell(1, nd); keep = false(1, nd);
for d = 1:nd
    if dropTm && strncmp(rb(d).TrueLabel, 'team', 4), continue, end
    [xy, ok] = nubotsMeasurement(rb(d).Box, rb(d).Class, p.CamPose, p.Cam, b.nubotsOpt);
    if ok, pos(:, d) = xy; keep(d) = true; end
    cls{d} = rb(d).Class;
end
pos = pos(:, keep); cls = cls(keep); R = repmat(b.R, 1, 1, numel(cls));
end

function [pos, R, cls] = readDetections(p, b, dropTm)
nd = numel(p.detections);
obs = p.ObserverPosition(:);
pos = zeros(2, nd); R = zeros(2, 2, nd); cls = cell(1, nd); keep = true(1, nd);
C = zeros(2); if isfield(p, 'ObserverCovariance') && ~isempty(p.ObserverCovariance), C = p.ObserverCovariance; end
hv = 0; if isfield(p, 'ObserverHeadingVariance') && ~isempty(p.ObserverHeadingVariance), hv = p.ObserverHeadingVariance; end
for d = 1:nd
    y = p.detections(d).y(:);
    u = y(1:2) / max(norm(y(1:2)), eps); rg = y(3);
    pos(:, d) = obs + rg*u;
    cls{d} = p.detections(d).Class;
    if dropTm && strncmp(p.detections(d).TrueLabel, 'team', 4), keep(d) = false; end
    if rg < b.selfExclude, keep(d) = false; end
    % Cartesian covariance of obs + r*u: bias C, tangential r^2(sd^2+hv)+t'Ct, radial sr^2+u'Cu
    tv = [-u(2); u(1)];
    R(:, :, d) = C + (rg^2*(p.DetectionNoiseStd^2 + hv) + tv.'*C*tv)*(tv*tv.') ...
        + (p.RangeNoiseStd^2 + u.'*C*u)*(u*u.');
end
pos = pos(:, keep); R = R(:, :, keep); cls = cls(keep);
end

%% ------------------------------------------------------------ faithful mode
function s = faithfulRobots(s, p, t, pos, cls, b, fl, fw)
obs = p.ObserverPosition(:);
seen = false(1, numel(s.act));
for d = find(strcmp(cls, 'robot'))
    z = pos(:, d);
    if isempty(s.act), best = 0; dist = inf;
    else
        pr = predPos(s.act, t);
        [dist, best] = min(hypot(pr(1, :) - z(1), pr(2, :) - z(2)));
    end
    if best == 0 || dist > b.gate
        s.counter = s.counter + 1;
        s.act(end+1) = newTrack(sprintf('robot%d', s.counter), 'robot', t, [z; 0; 0], ...
            diag([b.P0pos b.P0pos b.P0vel b.P0vel])); %#ok<AGROW>
        seen(end+1) = true; %#ok<AGROW>
    else
        s.act(best) = kfUpdate(s.act(best), t, z, b.R, b, 'robot');
        seen(best) = true;
    end
end
% maintenance
del = false(1, numel(s.act));
kept = [];
for j = 1:numel(s.act)
    a = s.act(j); pr = a.x(1:2, end) + a.x(3:4, end)*(t - a.t(end));
    inView = inFov(pr, obs, p);
    if ~inView, seen(j) = true; end
    if seen(j), a.misses = 0; else, a.misses = a.misses + 1; end
    if a.misses > b.maxMisses, del(j) = true; end
    if abs(pr(1)) > fl/2 + b.fieldMargin || abs(pr(2)) > fw/2 + b.fieldMargin, del(j) = true; end
    if b.proximityDelete && ~del(j)
        for q = kept
            pq = s.act(q).x(1:2, end) + s.act(q).x(3:4, end)*(t - s.act(q).t(end));
            if norm(pr - pq) < b.gate, del(j) = true; break; end
        end
    end
    if ~del(j), kept(end+1) = j; end %#ok<AGROW>
    s.act(j) = a;
end
s = retire(s, del, t);
end

function s = faithfulBall(s, t, bpos, b)
if isempty(bpos), return; end
if isempty(s.bF)
    if strcmp(b.ballInit, 'config'), x0 = [b.ballInitMean; 0; 0]; P0 = b.ballInitCov*eye(4);
    else, x0 = [bpos(:, 1); 0; 0]; P0 = diag([b.P0pos b.P0pos b.P0vel b.P0vel]); end
    s.bF = struct('x', x0, 'P', P0);
end
[d2, j] = min(sum((bpos - s.bF.x(1:2)).^2, 1));
z = bpos(:, j);
lowConf = false;
if d2 > b.ballAccept && ~s.firstSeen
    lowConf = true; s.rej = s.rej + 1;
else
    s.firstSeen = true; s.rej = 0;
end
if s.rej > b.ballMaxRej, s.rej = 0; end
[valid, avg] = teamAverage(s, t, b);
if b.useTeamBall && valid && lowConf
    s = emitBall(s, t, [avg; 0; 0], diag([b.teamBallStd^2 b.teamBallStd^2 1e-3 1e-3]), b);
else
    dt = max(t - s.tLast, 0); s.tLast = t;
    F = [eye(2) dt*eye(2); zeros(2) eye(2)];
    x = F*s.bF.x; P = F*s.bF.P*F.' + diag(baselineQ(b, 'ball'))*dt;
    H = [eye(2) zeros(2)]; Sg = H*P*H.' + b.R; K = P*H.'/Sg;
    x = x + K*(z - H*x); IKH = eye(4) - K*H;
    P = IKH*P*IKH.' + K*b.R*K.'; P = (P+P.')/2;
    s.bF.x = x; s.bF.P = P; s.lastBallUpd = t;
    s = emitBall(s, t, x, P, b);
end
end

function s = receiveGuess(s, from, t, pk, b)
s.gTime(from) = t; s.gPos(:, from) = pk;
if t - s.tLast < b.teamGuessDefaultTimer, return; end
[~, avg] = teamAverage(s, t, b);
s.tLast = t;
s = emitBall(s, t, [avg; 0; 0], diag([b.teamBallStd^2 b.teamBallStd^2 1e-3 1e-3]), b);
end

function [valid, avg] = teamAverage(s, t, b)
ok = (t - s.gTime) < b.teamBallRecency;
if ~any(ok), valid = false; avg = [0; 0]; return; end
g = s.gPos(:, ok); avg = mean(g, 2);
err = mean(sqrt(sum((g - avg).^2, 1)));
valid = err < b.teamBallStd;   % team_guess_error (0.5)
end

function s = emitBall(s, t, x, P, b) %#ok<INUSD>
if isempty(s.ball.t)
    s.ball = newTrack('ball1', 'ball', t, x, P); s.ball.hold = true;
else
    s.ball.t(end+1) = t; s.ball.x(:, end+1) = x; s.ball.P(:, :, end+1) = P;
end
end

%% --------------------------------------------------------------- tuned mode
function s = tunedStep(s, p, t, pos, R, cls, b, fl, fw)
obs = p.ObserverPosition(:);
for cl = {'robot', 'ball'}
    c = cl{1};
    di = find(strcmp(cls, c));
    ti = find(strcmp({s.act.class}, c));
    matchedD = false(1, numel(di));
    if ~isempty(ti) && ~isempty(di)
        D = inf(numel(ti), numel(di));
        for a = 1:numel(ti)
            tk = s.act(ti(a)); dt = t - tk.t(end);
            F = [eye(2) dt*eye(2); zeros(2) eye(2)];
            xp = F*tk.x(:, end); Pp = F*tk.P(:, :, end)*F.' + diag(baselineQ(b, c))*dt;
            for j = 1:numel(di)
                nu = pos(:, di(j)) - xp(1:2); Sg = Pp(1:2, 1:2) + R(:, :, di(j));
                d2 = nu.'/Sg*nu;
                if d2 <= b.chi2Gate, D(a, j) = d2; end
            end
        end
        while true
            [m, idx] = min(D(:));
            if ~isfinite(m), break; end
            [a, j] = ind2sub(size(D), idx);
            s.act(ti(a)) = kfUpdate(s.act(ti(a)), t, pos(:, di(j)), R(:, :, di(j)), b, c);
            matchedD(j) = true; D(a, :) = inf; D(:, j) = inf;
            if strcmp(c, 'ball'), break; end
        end
    end
    if strcmp(c, 'ball') && ~isempty(di)
        if ~isempty(ti) && any(matchedD), s.rej = 0;
        elseif ~isempty(ti)
            s.rej = s.rej + 1;
            if s.rej >= b.ballReacquire
                s = retire(s, ismember(1:numel(s.act), ti), t); s.rej = 0; ti = [];
            end
        end
        if isempty(ti)   % spawn from the closest-range unmatched detection
            j = find(~matchedD, 1);
            s = spawn(s, 'ball', t, pos(:, di(j)), R(:, :, di(j)), b);
        end
    elseif strcmp(c, 'robot')
        for j = find(~matchedD)
            s = spawn(s, 'robot', t, pos(:, di(j)), R(:, :, di(j)), b);
        end
    end
end
% age-out, field boundary, duplicate merge
n = numel(s.act); del = false(1, n); pr = zeros(2, n);
for j = 1:n
    a = s.act(j); pr(:, j) = a.x(1:2, end) + a.x(3:4, end)*(t - a.t(end));
    if t - a.t(end) > b.maxAge, del(j) = true; end
    if strcmp(a.class, 'robot') && (abs(pr(1, j)) > fl/2 + b.fieldMargin || abs(pr(2, j)) > fw/2 + b.fieldMargin)
        del(j) = true;
    end
end
ri = find(strcmp({s.act.class}, 'robot') & ~del);
for a = 1:numel(ri)
    for c2 = a+1:numel(ri)
        if del(ri(a)) || del(ri(c2)), continue; end
        if norm(pr(:, ri(a)) - pr(:, ri(c2))) < b.mergeDist
            if s.act(ri(a)).SpawnTime <= s.act(ri(c2)).SpawnTime, del(ri(c2)) = true; else, del(ri(a)) = true; end
        end
    end
end
s = retire(s, del, t);
end

function s = spawn(s, cl, t, z, Rz, b)
s.counter = s.counter + 1;
P0 = blkdiag(Rz, b.P0vel*eye(2));
s.act(end+1) = newTrack(sprintf('%s%d', cl, s.counter), cl, t, [z; 0; 0], P0);
end

%% ----------------------------------------------------------------- helpers
function s = retire(s, del, t)
if ~any(del), return; end
for j = find(del)
    s.act(j).alive = false; s.act(j).deletedAt = t;
end
s.done = [s.done, s.act(del)];
s.act = s.act(~del);
end

function pr = predPos(act, t)
n = numel(act); pr = zeros(2, n);
for j = 1:n, pr(:, j) = act(j).x(1:2, end) + act(j).x(3:4, end)*(t - act(j).t(end)); end
end

function v = inFov(pr, obs, p)
w = pr - obs; dist = norm(w);
ang = abs(mod(atan2(w(2), w(1)) - p.LookHeading + pi, 2*pi) - pi);
v = dist <= p.Range && ang <= p.FOV/2;
end

function s = newTrack(name, cl, t, x, P)
s = emptyTrack(); s.name = name; s.class = cl; s.t = t; s.x = x; s.P = P;
s.alive = true; s.SpawnTime = t;
end

function s = emptyTrack()
s = struct('name', '', 'class', '', 't', [], 'x', zeros(4,0), 'P', zeros(4,4,0), ...
    'alive', false, 'SpawnTime', 0, 'deletedAt', inf, 'misses', 0, 'hold', false);
end

function s = kfUpdate(s, t, z, Rm, b, cl)
dt = max(t - s.t(end), 0);
F = [eye(2) dt*eye(2); zeros(2) eye(2)];
x = F*s.x(:, end); P = F*s.P(:, :, end)*F.' + diag(baselineQ(b, cl))*dt;
H = [eye(2) zeros(2)];
Sg = H*P*H.' + Rm; K = P*H.'/Sg;
x = x + K*(z - H*x);
IKH = eye(4) - K*H;
P = IKH*P*IKH.' + K*Rm*K.'; P = (P+P.')/2;
s.t(end+1) = t; s.x(:, end+1) = x; s.P(:, :, end+1) = P;
end
