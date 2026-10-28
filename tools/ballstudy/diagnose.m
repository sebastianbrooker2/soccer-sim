% Diagnosis of ball tracking on real01 (shared tracker, default params). Needs JOINT_MAT (joint0.mat from run0).
root = fileparts(fileparts(fileparts(mfilename('fullpath')))); cd(root); addpath(genpath(root));
S = load(getenv('JOINT_MAT')); joint = S.joint; f = fastCopies(joint);
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
for r = 1:numel(joint)
    tr = joint(r).tracks; isb = strcmp({tr.Class}, 'ball');
    fprintf('copy %d: %d ball tracks ever, %d robot tracks; ball delete reasons: ', r, sum(isb), sum(~isb));
    dr = {tr(isb).DeleteReason}; u = unique(dr); for i = 1:numel(u), fprintf('%s=%d ', u{i}, sum(strcmp(dr,u{i}))); end; fprintf('\n');
    n = []; sd = []; sdOld = [];
    for tt = 1:0.5:gt.t(end)-1
        e = fastEstimates(f(r), tt, 'ball'); n(end+1) = numel(e); %#ok<SAGROW>
        for i = 1:numel(e), s = sqrt(max(eig(e(i).cov))); sd(end+1) = s; if tt - e(i).lastUpdate > 3, sdOld(end+1) = s; end, end %#ok<SAGROW>
    end
    fprintf('  live ball tracks: 0:%.1f%% 1:%.1f%% 2:%.1f%% 3+:%.1f%%  mean %.2f max %d\n', 100*mean(n==0), 100*mean(n==1), 100*mean(n==2), 100*mean(n>=3), mean(n), max(n));
    fprintf('  live ball std quantiles 10/50/90/99/max: %s ; stale>3s tracks: %d of %d samples, std 50/90/max: %s\n', mat2str(prctile(sd,[10 50 90 99 100]),3), numel(sdOld), numel(sd), mat2str(prctile(sdOld,[50 90 100]),3));
end
