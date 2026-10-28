function tests = test_export
tests = functiontests(localfunctions);
end

function testWriteAll(tc)
rng(2);
gt.t = (0:0.1:60)'; Nt = numel(gt.t); gt.ball = [cos(gt.t) sin(gt.t)];
gt.pos = zeros(Nt,2,6); gt.team = [1 1 1 2 2 2];
for k = 1:6, gt.pos(:,:,k) = [gt.t*0.1, k*ones(Nt,1)]; end
p = struct('evalDt', 0.1, 'radii', [0.25 0.5 1.0], 'tsStride', 2);
copies = struct('x', {1,2,3});
mkEst = @(s) @(c, te, cls) fakeEst(gt, te, cls, s);
pm = []; ts = cell(1,2);
for m = 1:2
    Tt = [evaluateMatch(gt, copies, mkEst(0.1), p, 'shared'); evaluateMatch(gt, copies, mkEst(0.4), p, 'baseline')];
    ts{m} = Tt; S = summarizeMatch(Tt, p); S.match = repmat(m, height(S), 1);
    pm = [pm; S]; %#ok<AGROW>
end
pm = movevars(pm, 'match', 'Before', 1);
res.perMatch = pm; res.timeseries = ts; res.params = p; res.summary = aggregateMatches(pm);
d = fullfile(tempname, 'out'); writeResults(res, d); writeLatex(res, d);
for f = {'per_match.csv','summary.csv','timeseries_match1.csv','timeseries_match2.csv','results.mat', ...
        'results_table.tex','fig_inregion_over_time.tex','fig_error_cdf.tex','main.tex','inregion_time.csv','err_cdf.csv'}
    tc.verifyTrue(isfile(fullfile(d, f{1})), f{1});
end
h = readtable(fullfile(d, 'per_match.csv'));
tc.verifyEqual(h.Properties.VariableNames(1:11), {'match','tracker','class','robot','inRegion95','within_0p25','within_0p5','within_1p0','rmse','availability','region_area_median'});
s = readtable(fullfile(d, 'summary.csv'));
tc.verifyEqual(s.Properties.VariableNames, {'tracker','class','metric','mean','std','nMatches'});
ht = readtable(fullfile(d, 'timeseries_match1.csv'));
tc.verifyEqual(ht.Properties.VariableNames, {'t','tracker','class','robot','target','err','inRegion95','hasTrack'});
tx = fileread(fullfile(d, 'results_table.tex')); tc.verifySubstring(tx, '\toprule');
disp(d);
end

function e = fakeEst(gt, te, cls, s)
if strcmp(cls, 'ball')
    e = struct('name', 'b', 'mean', interp1(gt.t, gt.ball, te)' + s*randn(2,1), 'cov', s^2*eye(2), 'class', 'ball');
else
    e = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
    for k = 4:6
        e(end+1) = struct('name', 'r', 'mean', interp1(gt.t, squeeze(gt.pos(:,:,k)), te)' + s*randn(2,1), 'cov', s^2*eye(2), 'class', 'robot'); %#ok<AGROW>
    end
end
end
