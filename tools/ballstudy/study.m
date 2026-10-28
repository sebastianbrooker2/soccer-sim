% Compare ball-reset options on real01 (or STUDY_GT=synth). Output: study_<name>.csv/.mat
root = fileparts(fileparts(fileparts(mfilename('fullpath')))); cd(root); addpath(genpath(root));
name = getenv('STUDY_NAME'); if isempty(name), name = 'real01'; end
tag = getenv('STUDY_TAG');
params = evalParams(); params.evalDt = 0.2;
if strcmp(name, 'real01')
    gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
    rng(1); packets = makePacketsYolo(gt, params, 'recordings/real01/yolo');
else
    gt = synthTeleportGT(str2double(name(6:end)));   % 'synthN'
    rng(1); packets = makePackets(gt, params);
end
numObs = numel(params.observers);
delivered = rand(numObs, numel(packets)) >= params.linkDropProb;
E = gtEvents(gt); if isfield(gt,'events'), E = gt.events; E.kind(E.kind=="jump") = "jump"; end
evT = sort([E.t(E.type=="ball" & ismember(E.kind, ["jump","exit","restart"])); ]);
bT = ballResetTimes(E); rM = robotResetMatrix(E);
fprintf('%s: %d packets, %d ball events, %d robot events\n', name, numel(packets), numel(bT), size(rM,1));
V = {};  % {label, struct of overrides}
V(end+1,:) = {'base', struct()};
V(end+1,:) = {'A_single', struct('ballSingle', true)};
V(end+1,:) = {'B_std3', struct('ballLostStd', 3)};
V(end+1,:) = {'B_std1.5', struct('ballLostStd', 1.5)};
V(end+1,:) = {'B_time3', struct('ballLostTime', 3)};
V(end+1,:) = {'C_N3', struct('ballResetN', 3)};
V(end+1,:) = {'C_N2', struct('ballResetN', 2)};
V(end+1,:) = {'C_N2_d2_v', struct('ballResetN', 2, 'ballResetD2', 40, 'ballResetVmax', 12)};
V(end+1,:) = {'D_ball', struct('ballResetTimes', bT)};
V(end+1,:) = {'R_robot', struct('robotResetEvents', rM)};
V(end+1,:) = {'D_ball+R', struct('ballResetTimes', bT, 'robotResetEvents', rM)};
V(end+1,:) = {'ABC', struct('ballSingle', true, 'ballLostStd', 3, 'ballResetN', 3)};
V(end+1,:) = {'ABC_vd', struct('ballSingle', true, 'ballLostStd', 3, 'ballResetN', 2, 'ballResetD2', 40, 'ballResetVmax', 12)};
V(end+1,:) = {'ABC+D+R', struct('ballSingle', true, 'ballLostStd', 3, 'ballResetN', 3, 'ballResetTimes', bT, 'robotResetEvents', rM)};
V(end+1,:) = {'S_range1', struct('spawnMaxRangeStd', 1.0)};
V(end+1,:) = {'S_pend', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true)};
V(end+1,:) = {'S_pend+repl', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballReplaceStd', 3)};
V(end+1,:) = {'S_pend+A+B', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 3, 'ballReplaceStd', 3)};
V(end+1,:) = {'S_pend+ABC', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 3, 'ballReplaceStd', 3, 'ballResetN', 3)};
V(end+1,:) = {'AB', struct('ballSingle', true, 'ballLostStd', 3)};
V(end+1,:) = {'FULL', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 3, 'ballReplaceStd', 3, 'ballResetN', 3)};
V(end+1,:) = {'FULL+D+R', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 3, 'ballReplaceStd', 3, 'ballResetN', 3, 'ballResetTimes', bT, 'robotResetEvents', rM)};
V(end+1,:) = {'FULL+R', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 3, 'ballReplaceStd', 3, 'ballResetN', 3, 'robotResetEvents', rM)};
V(end+1,:) = {'FULL8', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 8, 'ballReplaceStd', 8, 'ballResetN', 3)};
V(end+1,:) = {'FULL8+R', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 8, 'ballReplaceStd', 8, 'ballResetN', 3, 'robotResetEvents', rM)};
V(end+1,:) = {'FULLnoB', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballReplaceStd', 8, 'ballResetN', 3)};
V(end+1,:) = {'FULL20', struct('spawnMaxRangeStd', 0.6, 'spawnPending', true, 'ballSingle', true, 'ballLostStd', 20, 'ballReplaceStd', 8, 'ballResetN', 3)};
V(end+1,:) = {'E_kappa0.5', struct('kappaBall', 0.5)};
V(end+1,:) = {'E_acc1.5', struct('ballAccelStd', 1.5)};
V(end+1,:) = {'E_acc4', struct('ballAccelStd', 4)};
sel = getenv('STUDY_ONLY'); if ~isempty(sel), V = V(ismember(V(:,1), strsplit(sel, ',')), :); end
rows = {};
for i = 1:size(V, 1)
    p = params; f = fieldnames(V{i,2}); for k = 1:numel(f), p.tracker.(f{k}) = V{i,2}.(f{k}); end
    tic; cp = runSharedTracker(packets, delivered, p); tt = toc;
    rows(end+1,:) = row(['shared:' V{i,1}], gt, fastCopies(cp), @fastEstimates, p, evT, E, tt); %#ok<SAGROW>
    fprintf('%s done (%.0fs)\n', V{i,1}, tt); save(['tools/ballstudy/study_' name tag '.mat'], 'rows');
end
for md = {'faithful', 'tuned'}
    p = params; p.baseline.mode = md{1}; tic; cp = runBaselineTracker(packets, p); tt = toc;
    rows(end+1,:) = row(['base:' md{1}], gt, cp, @baselineEstimates, p, evT, E, tt); %#ok<SAGROW>
end
nm = {'b_avail','b_in95','b_in95cond','b_rmse','b_med','b_excess','b_multi','b_area','b_reacq','b_reacqFail','b_postIn','o_avail','o_in95','o_in95cond','o_rmse','o_false','o_reacq','o_postIn','secs'};
T = cell2table(rows(:,2:end), 'VariableNames', nm); T.label = string(rows(:,1)); T = movevars(T, 'label', 'Before', 1);
writetable(T, ['tools/ballstudy/study_' name tag '.csv']); disp(T);
function r = row(lab, gt, cp, fn, p, evT, E, tt)
M = ballMetrics(gt, cp, fn, p, evT); O = oppMetrics(M.Tall, E);
v = [M.availability M.inRegion95 M.inRegion95Cond M.rmse M.medErr M.excess M.fracMulti M.areaMedian M.reacq M.reacqFail M.postIn ...
     O.availability O.inRegion95 O.inRegion95Cond O.rmse O.falseTracks O.reacq O.postIn tt];
r = [{lab}, num2cell(v)];
end
