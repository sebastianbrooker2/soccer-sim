function results = runEvaluation(opts)
%RUNEVALUATION Compare per-robot baseline tracking against the cooperative
%   SRIF (square-root information form) tracker over whole matches.
%
%   results = runEvaluation(opts)   (see eval/SPEC.md)
%     opts.gtFiles  cell of ground-truth CSVs (spec schema; see
%                   tools/README_groundtruth.md). Empty -> synthetic matches.
%     opts.nMatches number of matches (default: numel(gtFiles), else 10)
%     opts.duration seconds per match (default 1200; synthetic only, and a
%                   cap on real files when given)
%     opts.seed     base RNG seed (default 1)
%     opts.runName  output folder name under eval/results (default timestamp)
%     opts.params   evalParams() overrides, struct merged field by field
%     opts.packetSource 'default' (params.sensor.model) | 'yolo' (logged detector
%                   output via makePacketsYolo)
%     opts.yoloRoot dir holding realNN/yolo/dets.csv (default <repo>/recordings);
%                   realNN taken from the gt file name (real03.csv -> real03)
%     opts.gtDt     ground-truth grid step in s (default 0.05)
%
%   Writes eval/results/<runName>/{per_match,summary}.csv, timeseries CSVs,
%   results.mat, results_table.tex, fig_*.tex, main.tex.

if nargin < 1, opts = struct(); end
here = fileparts(mfilename('fullpath'));
addpath(genpath(fileparts(here)));
opt = @(n, d) getOr(opts, n, d);

gtFiles = opt('gtFiles', {});
if ischar(gtFiles) || isstring(gtFiles), gtFiles = cellstr(gtFiles); end
nMatches = opt('nMatches', max(numel(gtFiles), 10 * isempty(gtFiles)));
duration = opt('duration', 1200);
seed     = opt('seed', 1);
gtDt     = opt('gtDt', 0.05);
runName  = opt('runName', ['run_' datestr(now, 'yyyymmdd_HHMMSS')]); %#ok<TNOW1,DATST>
outDir   = fullfile(here, 'results', runName);
yolo.source = opt('packetSource', 'default');
yolo.root   = opt('yoloRoot', fullfile(fileparts(here), 'recordings'));

params = mergeStruct(evalParams(), opt('params', struct()));

trackers = {'baseline', @runBaselineTracker, @baselineEstimates; ...
            'baseline_tuned', @runBaselineTrackerTuned, @baselineEstimates; ...
            'shared',   [],                  @trackEstimates};

Ss = cell(1, nMatches);
timeseries = cell(1, nMatches);
runtimes = zeros(nMatches, 2);
usePar = getOr(opts, 'parallel', true) && nMatches > 1 && ~isempty(ver('parallel'));
if usePar && isempty(gcp('nocreate')), parpool('Processes', min(nMatches, 10)); end
if usePar, M = Inf; else, M = 0; end
parfor (k = 1:nMatches, M)
    [Ss{k}, timeseries{k}, runtimes(k, :)] = runOneMatch(k, nMatches, seed, gtFiles, duration, gtDt, params, trackers, yolo);
end
perMatch = table();
for k = 1:nMatches
    perMatch = [perMatch; Ss{k}]; %#ok<AGROW>
    printMatch(Ss{k});
end

results.perMatch = perMatch;
results.timeseries = timeseries;
results.summary = aggregateMatches(perMatch);
results.params = params;
results.opts = opts;
results.runtimes = runtimes;  % [tracker seconds, total seconds] per match
fprintf('per-match total seconds: %s\n', mat2str(round(runtimes(:,2)')));

writeResults(results, outDir);
writeLatex(results, outDir);
fprintf('\nResults written to %s\n', outDir);
disp(results.summary);
end

function [S, T, rt] = runOneMatch(k, nMatches, seed, gtFiles, duration, gtDt, params, trackers, yolo)
t00 = tic;
fprintf('=== match %d/%d ===\n', k, nMatches);
rng(seed + k);
if isempty(gtFiles)
    gt = synthGroundTruth(duration, gtDt, seed + k);
else
    gt = loadGroundTruth(gtFiles{mod(k - 1, numel(gtFiles)) + 1}, gtDt);
    gt = cropGT(gt, duration);
end
if strcmpi(yolo.source, 'yolo')
    [~, nm] = fileparts(gtFiles{mod(k - 1, numel(gtFiles)) + 1});
    packets = makePacketsYolo(gt, params, fullfile(yolo.root, nm, 'yolo'));
else
    packets = makePackets(gt, params);
end
numObs = numel(params.observers);
delivered = rand(numObs, numel(packets)) >= params.linkDropProb;
fprintf('  [m%d] %d packets over %.0f s\n', k, numel(packets), gt.t(end) - gt.t(1));
T = table(); trk = 0;
for i = 1:size(trackers, 1)
    name = trackers{i, 1};
    tic;
    if strcmp(name, 'shared')
        copies = runSharedTracker(packets, delivered, params);
    else
        copies = trackers{i, 2}(packets, params);
    end
    el = toc; trk = trk + el;
    fprintf('  [m%d] %-8s tracker: %.1f s\n', k, name, el);
    T = [T; evaluateMatch(gt, copies, trackers{i, 3}, params, name)]; %#ok<AGROW>
end
S = summarizeMatch(T, params);
S.match = repmat(k, height(S), 1);
rt = [trk, toc(t00)];
fprintf('  [m%d] done in %.0f s\n', k, rt(2));
end

function printMatch(S)
S = S(ismember(S.class, {'ball', 'opponent'}), :);
G = groupsummary(S, {'tracker', 'class'}, 'mean', 'inRegion95');
for i = 1:height(G)
    fprintf('  %-8s %-9s in-95%%-region: %5.1f%%\n', G.tracker{i}, G.class{i}, 100 * G.mean_inRegion95(i));
end
end

function gt = cropGT(gt, duration)
keep = gt.t - gt.t(1) <= duration;
gt.t = gt.t(keep); gt.ball = gt.ball(keep, :);
gt.pos = gt.pos(keep, :, :); gt.theta = gt.theta(keep, :);
end

function v = getOr(s, n, d)
if isfield(s, n) && ~isempty(s.(n)), v = s.(n); else, v = d; end
end

function a = mergeStruct(a, b)
f = fieldnames(b);
for i = 1:numel(f)
    if isstruct(b.(f{i})) && isfield(a, f{i}) && isstruct(a.(f{i}))
        a.(f{i}) = mergeStruct(a.(f{i}), b.(f{i}));
    else
        a.(f{i}) = b.(f{i});
    end
end
end
