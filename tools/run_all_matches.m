function run_all_matches(varargin)
% RUN_ALL_MATCHES  Run all trackers on the shipped YOLO-detection matches and average the metrics.
%   run_all_matches                       all 10 matches (real01..real10), resume (skip finished), serial
%   run_all_matches('matches', 2)         only real02          ('matches', [1 5 7] = subset)
%   run_all_matches('parallel', true)     parfor over the 3 'own' SRIF runs per match (needs Parallel Toolbox)
%   run_all_matches('resume', false)      recompute matches even if tools/results_all/match_realNN.csv exists
%   run_all_matches('seed', 1)            packet rng seed; match k uses rng(seed+k-1)  (real01 -> rng(1), as run_compare3)
%   run_all_matches('outDir', d)          output folder (default tools/results_all)
% Behaviours per match (same packets/GT/0.2 s grid/one-to-one matching/maxAreaForScore 3 as tools/compare3):
%   NUbots (faithful), Tuned baseline, Own SRIF, Joint SRIF, Raw measurement.
% Writes match_realNN.csv per match, then per_match.csv (all finished matches) and summary.csv (mean/std over matches).
% A match missing eval/gt/realNN.csv or recordings/realNN/yolo/dets.csv is skipped with a message.
p = inputParser;
addParameter(p, 'matches', 1:10); addParameter(p, 'parallel', false); addParameter(p, 'resume', true);
addParameter(p, 'seed', 1); addParameter(p, 'outDir', '');
parse(p, varargin{:}); o = p.Results;
here = fileparts(mfilename('fullpath')); root = fileparts(here);
cd(root); addpath(genpath(root));
if isempty(o.outDir), o.outDir = fullfile(here, 'results_all'); end
if ~exist(o.outDir, 'dir'), mkdir(o.outDir); end

for k = o.matches(:).'
    nm = sprintf('real%02d', k);
    f = fullfile(o.outDir, ['match_' nm '.csv']);
    gtf = fullfile(root, 'eval', 'gt', [nm '.csv']); ydir = fullfile(root, 'recordings', nm, 'yolo');
    if o.resume && exist(f, 'file'), fprintf('[%s] result exists, skipping (resume)\n', nm); continue, end
    if ~exist(gtf, 'file') || ~exist(fullfile(ydir, 'dets.csv'), 'file')
        fprintf('[%s] MISSING ground truth or detections (%s, %s) - skipped\n', nm, gtf, ydir); continue
    end
    t0 = tic;
    try
        R = runOne(gtf, ydir, o.seed + k - 1, o.parallel);
    catch ME
        fprintf('[%s] FAILED: %s\n', nm, ME.message); continue
    end
    R = addvars(R, repmat(string(nm), height(R), 1), 'Before', 1, 'NewVariableNames', 'match');
    writetable(R, f);
    fprintf('[%s] done in %.0f s -> %s\n', nm, toc(t0), f);
end

% ---- aggregate every finished match file
fs = dir(fullfile(o.outDir, 'match_real*.csv'));
if isempty(fs), disp('no match results to aggregate'); return, end
A = table();
for i = 1:numel(fs), A = [A; readtable(fullfile(fs(i).folder, fs(i).name), 'TextType', 'string')]; end %#ok<AGROW>
writetable(A, fullfile(o.outDir, 'per_match.csv'));
metrics = A.Properties.VariableNames(6:end);   % after match,tracker,class,scope,n
S = A(A.scope == "all", :);
[g, trk, cls] = findgroups(S.tracker, S.class);
rows = table(); nM = splitapply(@numel, S.match, g);
for mi = 1:numel(metrics)
    v = S.(metrics{mi});
    mu = splitapply(@(x) mean(x, 'omitnan'), v, g); sd = splitapply(@(x) std(x, 'omitnan'), v, g);
    rows = [rows; table(trk, cls, repmat(string(metrics{mi}), numel(trk), 1), nM, mu, sd, ...
        'VariableNames', {'tracker','class','metric','n_matches','mean','std'})]; %#ok<AGROW>
end
writetable(rows, fullfile(o.outDir, 'summary.csv'));
fprintf('aggregated %d matches -> per_match.csv, summary.csv\n', numel(fs));
key = ["availability","inRegion95","inRegion95_capped","rmse","median_err","false_tracks_per_sample"];
disp(rows(ismember(rows.metric, key), :));
end

function R = runOne(gtf, ydir, seed, par)
params = evalParams(); params.baseline.mode = 'faithful';
params.evalDt = 0.2; params.maxAreaForScore = 3;
gt = loadGroundTruth(gtf, 0.05);
rng(seed);
packets = makePacketsYolo(gt, params, ydir);
numObs = numel(params.observers);
delivered = rand(numObs, numel(packets)) >= params.linkDropProb;
rid = arrayfun(@(p) p.RobotID, packets);
C = struct();
tic; C.nubots = runBaselineTracker(packets, params); fprintf('  nubots %.0fs\n', toc);
pt = params; pt.baseline.mode = 'tuned';
tic; C.tuned = runBaselineTracker(packets, pt); fprintf('  tuned %.0fs\n', toc);
tic; C.joint = runSharedTracker(packets, delivered, params); fprintf('  joint %.0fs\n', toc);
own = cell(1, numObs);
if par
    parfor r = 1:numObs
        dm = false(numObs, numel(packets)); dm(r, rid == r) = true;
        sr = runSharedTracker(packets, dm, params); own{r} = sr(r);
    end
else
    for r = 1:numObs
        dm = false(numObs, numel(packets)); dm(r, rid == r) = true;
        tic; sr = runSharedTracker(packets, dm, params); fprintf('  own r%d %.0fs\n', r, toc); own{r} = sr(r);
    end
end
C.own = [own{:}];
C.raw = runRawMeasurement(packets, params);
names = {'nubots','own','joint','raw','tuned'}; labels = {'NUbots (faithful)','Own SRIF','Joint SRIF','Raw measurement','Tuned baseline'};
nTrue = struct('ball', 1, 'opponent', 3); rows = {};
for i = 1:numel(names)
    n = names{i};
    if strcmp(n, 'raw'), fn = @(c,t,cl) rawEstimates(c,t,cl,params); fc = C.(n);
    elseif any(strcmp(n, {'nubots','tuned'})), fn = @(c,t,cl) baselineEstimates(c,t,cl); fc = C.(n);
    else, fn = @fastEstimates; fc = fastCopies(C.(n)); end
    S0 = evaluateMatch(gt, fc, fn, params, n);
    [g_, ~] = findgroups(S0.robot, S0.t, S0.class); sh_ = splitapply(@sum, double(S0.hasTrack), g_); S0.falseTracks = S0.nTracks - sh_(g_);
    for cl = {'ball','opponent'}
        for sc = 0:numObs
            S = S0(S0.class == cl{1}, :);
            if sc > 0, S = S(S.robot == sc, :); end
            h = S.hasTrack; ne = S.nees(h & isfinite(S.nees)); w = S.withinR; nl = S.nTracks;
            if sc == 0, scs = 'all'; else, scs = sprintf('robot%d', sc); end
            rows(end+1,:) = {labels{i}, cl{1}, scs, height(S), mean(h), mean(S.inRegion95), mean(S.inRegion95(h)), ...
                mean(S.inRegion95Capped), mean(S.inRegion95Capped(h)), mean(w(:,2)), mean(w(h,2)), sqrt(mean(S.err(h).^2)), ...
                median(S.err(h)), median(S.area, 'omitnan'), mean(ne), median(ne), mean(nl), mean(nl) - nTrue.(cl{1}), mean(S.falseTracks)}; %#ok<AGROW>
        end
    end
end
vn = {'tracker','class','scope','n','availability','inRegion95','inRegion95_cond','inRegion95_capped','inRegion95_capped_cond', ...
      'within05','within05_cond','rmse','median_err','median_area95','mean_nees_cond','median_nees_cond','mean_live_tracks','live_minus_true','false_tracks_per_sample'};
R = cell2table(rows, 'VariableNames', vn);
end
