function tests = test_metrics
tests = functiontests(localfunctions);
end

function [gt, params] = mk(n)
rng(1);
dt = 0.1; t = (0:dt:n*dt)';
gt.t = t; Nt = numel(t);
gt.ball = [cos(t) sin(t)];
gt.pos = zeros(Nt,2,6);
for k = 1:6, gt.pos(:,:,k) = [t*0.1+k, -k*ones(Nt,1)]; end
gt.team = [1 1 1 2 2 2];
params = struct('evalDt', dt, 'radii', [0.25 0.5 1.0], 'evalTeammates', false);
end

function f = noisyEst(gt, sigTrue, sigClaim)
% estimator returning truth + N(0,sigTrue^2) with claimed cov sigClaim^2*I
f = @(copy, te, cls) est(gt, te, cls, sigTrue, sigClaim);
end

function e = est(gt, te, cls, sT, sC)
if strcmp(cls, 'ball')
    p = interp1(gt.t, gt.ball, te);
    e = struct('name', 'b', 'mean', p' + sT*randn(2,1), 'cov', sC^2*eye(2), 'class', 'ball');
else
    e = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
    for k = 4:6
        p = interp1(gt.t, squeeze(gt.pos(:,:,k)), te);
        e(end+1) = struct('name', sprintf('r%d', k), 'mean', p' + sT*randn(2,1), 'cov', sC^2*eye(2), 'class', 'robot'); %#ok<AGROW>
    end
end
end

function testConsistent(tc)
[gt, p] = mk(4000); copies = struct('x', {1,2,3});
T = evaluateMatch(gt, copies, noisyEst(gt, 0.3, 0.3), p, 'cons');
tc.verifyEqual(height(T), 3*numel(gt.t)*4);
tc.verifyEqual(mean(T.inRegion95), 0.95, 'AbsTol', 0.01);
tc.verifyGreaterThan(mean(T.hasTrack), 0.99);  % rare swaps with nearby teammates are legitimate
S = summarizeMatch(T, p);
tc.verifyTrue(ismember('within_1p0', S.Properties.VariableNames));
tc.verifyEqual(mean(S.region_area_median), pi*(-2*log(0.05))*0.09, 'RelTol', 1e-6);
end

function testOverconfident(tc)
[gt, p] = mk(2000); copies = struct('x', {1,2,3});
T = evaluateMatch(gt, copies, noisyEst(gt, 0.3, 0.05), p, 'over');
tc.verifyLessThan(mean(T.inRegion95), 0.1);
end

function testNoTrack(tc)
[gt, p] = mk(50); copies = struct('x', {1,2,3});
none = @(c, te, cls) struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
T = evaluateMatch(gt, copies, none, p, 'none');
tc.verifyFalse(any(T.hasTrack)); tc.verifyFalse(any(T.inRegion95));
tc.verifyTrue(all(isnan(T.err)));
S = summarizeMatch(T, p);
tc.verifyEqual(S.availability, zeros(height(S),1));
tc.verifyTrue(all(isnan(S.rmse)));
end

function testTeammatesAndAggregate(tc)
[gt, p] = mk(100); p.evalTeammates = true; copies = struct('x', {1,2,3});
T = evaluateMatch(gt, copies, noisyEst(gt, 0.1, 0.1), p, 'a');
tc.verifyEqual(sum(T.class == "teammate" & T.robot == 1), 2*numel(unique(T.t)));
p.evalTeammates = false;
rows = [];
for m = 1:3
    T = evaluateMatch(gt, copies, noisyEst(gt, 0.1, 0.1), p, 'a');
    S = summarizeMatch(T, p); S.match = repmat(m, height(S), 1);
    rows = [rows; S]; %#ok<AGROW>
end
A = aggregateMatches(rows);
tc.verifyEqual(unique(A.nMatches), 3);
tc.verifyTrue(any(A.metric == "inRegion95" & A.class == "ball"));
end

function testGhostTracksDoNotInflate(tc)
% one huge-covariance ghost covers all 3 opponents in nearest mode (gamed);
% one-to-one lets it serve a single target; the area cap removes it entirely.
[gt, p] = mk(300); copies = struct('x', {1,2,3});
f = @(c, te, cls) ghostFcn(gt, te, cls, 1);
p.matchMode = 'nearest';  Tn = evaluateMatch(gt, copies, f, p, 'g');
p.matchMode = 'onetoone'; To = evaluateMatch(gt, copies, f, p, 'g');
oppN = Tn(Tn.class == "opponent", :); oppO = To(To.class == "opponent", :);
tc.verifyEqual(mean(oppN.inRegion95), 1, 'AbsTol', 1e-9);
tc.verifyEqual(mean(oppO.inRegion95), 1/3, 'AbsTol', 1e-9);
% many ghosts, no real tracks: false tracks and area cap expose them
f = @(c, te, cls) ghostFcn(gt, te, cls, 20);
p.maxAreaForScore = 5; To = evaluateMatch(gt, copies, f, p, 'g');
S = summarizeMatch(To, p); So = S(S.class == "opponent", :);
tc.verifyEqual(mean(So.falseTracks), 20 - 5, 'AbsTol', 1e-9);   % 20 live - 5 true robots
tc.verifyGreaterThan(mean(So.inRegion95), 0.5);
tc.verifyEqual(mean(So.inRegion95_capped), 0, 'AbsTol', 1e-9);
tc.verifyGreaterThan(mean(So.region_area_recent_median), 1000);
tc.verifyTrue(all(ismember({'inRegion95_cond','within_0p5_cond','nees_mean'}, S.Properties.VariableNames)));
end

function e = ghostFcn(gt, te, cls, nGhost)
e = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
if strcmp(cls, 'ball'), return; end
c = mean(squeeze(interp1(gt.t, permute(gt.pos(:,:,4:6), [1 3 2]), te)), 1)';
for g = 1:nGhost
    e(end+1) = struct('name', 'ghost', 'mean', c + 0.01*g*[1;0], 'cov', 100*eye(2), 'class', 'robot'); %#ok<AGROW>
end
end

function testOneToOneBasics(tc)
% a single track near two targets can only score one of them
[gt, p] = mk(20); copies = struct('x', {1});
f = @(c, te, cls) oneTrack(gt, te, cls);
T = evaluateMatch(gt, copies, f, p, 'o');
opp = T(T.class == "opponent", :);
tc.verifyEqual(mean(opp.hasTrack), 1/3, 'AbsTol', 1e-9);
end

function e = oneTrack(gt, te, cls)
e = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
if strcmp(cls, 'ball'), return; end
p = interp1(gt.t, squeeze(gt.pos(:,:,4)), te)';
e = struct('name', 'x', 'mean', p, 'cov', eye(2), 'class', 'robot');
end
