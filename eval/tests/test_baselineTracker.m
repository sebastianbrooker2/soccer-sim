function test_baselineTracker()
rng(3);
T = 20;
[pk, v] = makeScene(T);
nb = struct('useTeamBall', false);

%% faithful: ball filter fuses, converges (init (5,0) is wiped by the large first dt)
pF = struct('baseline', nb);
c = runBaselineTracker(pk, pF);
e = baselineEstimates(c(1), T, 'ball');
assert(numel(e) == 1, 'one ball track');
err = norm(e.mean - ([3;0] + v*T)); assert(err < 0.3, 'faithful convergence err %.3f', err);
% hold: no prediction between measurements
pk1 = pk(arrayfun(@(q) q.RobotID == 1, pk)); pk1 = pk1(1:100);
c1 = runBaselineTracker(pk1, pF);
ea = baselineEstimates(c1(1), pk1(end).time, 'ball'); eb = baselineEstimates(c1(1), pk1(end).time + 5, 'ball');
assert(isequal(ea.mean, eb.mean) && isequal(ea.cov, eb.cov), 'faithful ball must not predict between measurements');

%% tuned
pT = struct('baseline', struct('mode', 'tuned'));
ct = runBaselineTracker(pk, pT);
e = baselineEstimates(ct(1), T, 'ball');
assert(numel(e) == 1 && norm(e.mean - ([3;0] + v*T)) < 0.3, 'tuned convergence');
assert(trace(e.cov) < 0.1, 'tuned cov plausible');
% ages out after maxAge
pk5 = pk1(1:60);
c5 = runBaselineTracker(pk5, pT);
tl = pk5(60).time;
assert(~isempty(baselineEstimates(c5(1), tl, 'ball')));
for i = 1:2, pk5(end+1) = pk5(60); pk5(end).time = tl + 4*i; pk5(end).detections(:) = []; end %#ok<AGROW>
c5 = runBaselineTracker(pk5, pT);
assert(isempty(baselineEstimates(c5(1), tl + 8, 'ball')), 'tuned ball ages out');

%% no truth leakage (other than teammate filter): scramble labels
pk2 = pk; for i = 1:numel(pk2), pk2(i).detections(1).TrueLabel = 'zzz'; end
for P = {pF, pT}
    a = runBaselineTracker(pk, P{1}); b2 = runBaselineTracker(pk2, P{1});
    assert(isequal(a(1).tracks(1).x, b2(1).tracks(1).x), 'truth leakage');
end

%% isolation: robot 1 must not use robot 2's packets (team ball off)
for P = {pF, pT}
    a = runBaselineTracker(pk, P{1}); a3 = runBaselineTracker(pk(arrayfun(@(q) q.RobotID == 1, pk)), P{1});
    assert(isequal(a(1).tracks(1).x, a3(1).tracks(1).x), 'sharing');
end
cc = runBaselineTracker(pk, pT);
assert(abs(cc(2).tracks(1).x(1,1) - 99) < 1, 'robot 2 isolated');

%% teammate detections ignored when dropTeammateDetections
ptm = pk(arrayfun(@(q) q.RobotID == 1, pk)); ptm = ptm(1:50);
for i = 1:numel(ptm)
    ptm(i).detections(2) = struct('y', [1;0;3], 'Class', 'robot', 'TrueLabel', 'team2', 'LastDetection', []);
end
cd0 = runBaselineTracker(ptm, struct('dropTeammateDetections', true, 'baseline', nb));
cd1 = runBaselineTracker(ptm, struct('dropTeammateDetections', false, 'baseline', nb));
assert(isempty(baselineEstimates(cd0(1), 4, 'robot')) && ~isempty(baselineEstimates(cd1(1), 4, 'robot')), 'teammate drop');

%% faithful robot rules
% (a) moving robot faster than the 0.5 m association radius spawns ghosts; (b) deleted after >25 misses in view
pr = pk(arrayfun(@(q) q.RobotID == 1, pk)); pr = pr(1:150);
for i = 1:numel(pr)
    z = [2; 0.2*pr(i).time] ; w = z - pr(i).ObserverPosition; r = norm(w);
    pr(i).detections = struct('y', [w/r; r], 'Class', 'robot', 'TrueLabel', 'opp1', 'LastDetection', []);
    if i > 100, pr(i).detections(:) = []; end
end
cr = runBaselineTracker(pr, struct('baseline', nb));
assert(numel(cr(1).tracks) == 1 && ~cr(1).tracks(1).alive, 'robot deleted after misses in view (>25)');
assert(abs(cr(1).tracks(1).deletedAt - pr(126).time) < 1e-9, 'deleted on the 26th consecutive in-view miss');
% out of view: never missed -> persists
po = pr; for i = 101:numel(po), po(i).LookHeading = pi; end
co = runBaselineTracker(po, struct('baseline', nb));
assert(co(1).tracks(1).alive, 'out-of-view tracks are not aged');
% outside field+1 m is deleted
pf = pr(1:5); for i = 1:5, pf(i).detections = struct('y', [1;0;9], 'Class', 'robot', 'TrueLabel', 'opp1', 'LastDetection', []); end
cf = runBaselineTracker(pf, struct('baseline', nb));
assert(isempty(baselineEstimates(cf(1), pf(5).time, 'robot')), 'outside field deleted');

%% baselineEstimates honours params (Q) and stored copy params
cq = runBaselineTracker(pr(1:30), struct('baseline', nb));
e0 = baselineEstimates(cq(1), pr(30).time + 2, 'robot');
e1 = baselineEstimates(cq(1), pr(30).time + 2, 'robot', struct('baseline', struct('QoppPos', 5)));
assert(trace(e1.cov) > trace(e0.cov) + 5, 'params honoured');

%% team ball fallback: robot 2 sees the ball, robot 1 does not -> robot 1 gets it from the team
pt = pk; for i = 1:numel(pt), if pt(i).RobotID == 1 && pt(i).time > 5, pt(i).detections(:) = []; end
    if pt(i).RobotID == 2, pt(i).detections(1).y = pk(i-1).detections(1).y; end, end
ctb = runBaselineTracker(pt, struct('linkDropProb', 0));
assert(~isempty(baselineEstimates(ctb(1), 18, 'ball')), 'team ball fallback provides an estimate');
disp('test_baselineTracker passed');
end

function [pk, v] = makeScene(T)
pk = struct('time', {}, 'RobotID', {}, 'ObserverPosition', {}, 'ObserverCovariance', {}, ...
    'DetectionNoiseStd', {}, 'RangeNoiseStd', {}, 'LookHeading', {}, 'FOV', {}, 'Range', {}, 'detections', {});
obs = [0; 0]; v = [0.5; 0.2];
for t = 0.1:0.1:T
    ball = [3; 0] + v*t;
    d = ball - obs; r = norm(d); u = d/r;
    y = [u + 0.01*randn(2,1); r + 0.05*randn];
    p = struct('time', t, 'RobotID', 1, 'ObserverPosition', obs, 'ObserverCovariance', 0.01*eye(2), ...
        'DetectionNoiseStd', 0.01, 'RangeNoiseStd', 0.05, 'LookHeading', 0, 'FOV', 2, 'Range', 8, ...
        'detections', struct('y', y, 'Class', 'ball', 'TrueLabel', 'ball', 'LastDetection', []));
    pk(end+1) = p; %#ok<AGROW>
    p.RobotID = 2; p.detections(1).y = [1;0;99]; pk(end+1) = p; %#ok<AGROW> robot 2 sees junk
end
end
