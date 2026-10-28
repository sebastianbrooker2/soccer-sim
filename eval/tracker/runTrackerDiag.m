function R = runTrackerDiag(variants, seeds, duration)
%RUNTRACKERDIAG Synthetic matches -> makePackets -> runSharedTracker -> trackerDiagnostics.
%   variants: struct, or cell array of structs, of overrides (tracker params at
%   top level of each struct; fields named like evalParams top-level, e.g.
%   dropTeammateDetections, go there). Runs variants x seeds in one parfor and
%   prints the per-variant mean over seeds. EVALUATION ONLY (reads truth).
if nargin < 1, variants = {struct()}; end
if isstruct(variants), variants = {variants}; end
if nargin < 2, seeds = 1:3; end
if nargin < 3, duration = 300; end
nV = numel(variants); nS = numel(seeds);
base = evalParams();
topFields = fieldnames(base);
Ds = cell(nV*nS, 1); tt = zeros(nV*nS, 1);
parfor c = 1:nV*nS
    v = ceil(c/nS); s = seeds(mod(c-1, nS) + 1);
    params = base;
    ov = variants{v};
    params = applyOverrides(params, ov, topFields);
    rng(s);
    gt = synthGroundTruth(duration, 0.05, s);
    pk = makePackets(gt, params);
    dl = rand(numel(params.observers), numel(pk)) >= params.linkDropProb;
    tic; cp = runSharedTracker(pk, dl, params); tt(c) = toc;
    Ds{c} = trackerDiagnostics(gt, cp, params);
end
R = struct2table([Ds{:}]);
R.sec_per_simmin = tt / duration * 60;
R.variant = repelem((1:nV).', nS, 1);
M = varfun(@(x) mean(x, 'omitnan'), R, 'GroupingVariables', 'variant');
M.GroupCount = []; M.Properties.VariableNames = regexprep(M.Properties.VariableNames, '^Fun_', '');
disp(M)
end

function params = applyOverrides(params, ov, topFields)
fn = fieldnames(ov);
for i = 1:numel(fn)
    if any(strcmp(fn{i}, topFields)) && ~isstruct(ov.(fn{i})), params.(fn{i}) = ov.(fn{i});
    else, params.tracker.(fn{i}) = ov.(fn{i}); end
end
end
