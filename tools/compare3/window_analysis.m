% Per-time-window comparison joint vs own SRIF (reads compare3.mat, logs.mat produced by run_compare3.m).
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'compare3.mat')); L = load(fullfile(here, 'logs.mat'));
W = 20; out = {};
for cl = {'ball', 'opponent'}
    J = S.T.joint(S.T.joint.class == cl{1}, :); O = S.T.own(S.T.own.class == cl{1}, :);
    edges = floor(min(J.t)/W)*W : W : ceil(max(J.t)/W)*W;
    for i = 1:numel(edges)-1
        mj = J.t >= edges(i) & J.t < edges(i+1); mo = O.t >= edges(i) & O.t < edges(i+1);
        if ~any(mj), continue, end
        g = @(X, m) mean(X.hasTrack(m) & X.err(m) < 1);        % tracked within 1 m
        c = @(X, m) mean(X.inRegion95(m));
        e = @(X, m) median(X.err(m & X.hasTrack), 'omitnan');
        out(end+1,:) = {cl{1}, edges(i), g(J,mj), g(O,mo), c(J,mj), c(O,mo), mean(J.hasTrack(mj)), mean(O.hasTrack(mo)), e(J,mj), e(O,mo), mean(J.nTracks(mj)), mean(O.nTracks(mo))}; %#ok<AGROW>
    end
end
Wt = cell2table(out, 'VariableNames', {'class','t0','joint_within1m','own_within1m','joint_in95','own_in95','joint_avail','own_avail','joint_medErr','own_medErr','joint_live','own_live'});
Wt.class = string(Wt.class); Wt.delta = Wt.joint_within1m - Wt.own_within1m;
writetable(Wt, fullfile(here, 'windows.csv'));
for cl = {'ball', 'opponent'}
    X = Wt(Wt.class == cl{1}, :);
    fprintf('%s: joint better in %d / %d windows, worse in %d (delta<-0.05); mean delta %.3f\n', cl{1}, sum(X.delta > 0.05), height(X), sum(X.delta < -0.05), mean(X.delta));
    X = sortrows(X, 'delta'); disp(X(1:8, :));
end
