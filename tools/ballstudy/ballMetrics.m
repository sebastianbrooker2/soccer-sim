function M = ballMetrics(gt, copies, estFcn, params, evTimes, win)
%BALLMETRICS Ball-only metrics for one tracker over all observer copies.
%   M: availability, inRegion95 (all samples), inRegion95Cond (given a track),
%   rmse, medErr, meanLive (mean live ball tracks), excess (mean max(live-1,0)),
%   fracMulti (frac of time >1 live), areaMean/areaMedian (95% region m^2),
%   reacq (median/mean time to reacquire after evTimes, s), reacqFail (count),
%   postErr (median err within [0,win] s after each event), plus T (table).
if nargin < 6, win = 5; end
Tall = evaluateMatch(gt, copies, estFcn, params, 'x');
T = Tall(Tall.class == "ball", :);
M.n = height(T);
M.availability = mean(T.hasTrack);
M.inRegion95 = mean(T.inRegion95);
M.inRegion95Cond = mean(T.inRegion95(T.hasTrack));
M.rmse = sqrt(mean(T.err(T.hasTrack).^2));
M.medErr = median(T.err(T.hasTrack));
M.meanLive = mean(T.nTracks);
M.excess = mean(max(T.nTracks - 1, 0));
M.fracMulti = mean(T.nTracks > 1);
M.areaMean = mean(T.area, 'omitnan');
M.areaMedian = median(T.area, 'omitnan');
M.within1 = mean(T.withinR(:, 3));
M.reacq = nan; M.reacqMean = nan; M.reacqFail = 0; M.postErr = nan; M.postIn = nan; M.nEv = numel(evTimes);
if ~isempty(evTimes)
    tt = [];  pe = []; pi_ = [];
    robs = unique(T.robot);
    evs = evTimes(:); nxt = [evs(2:end); inf];
    for r = robs'
        Tr = T(T.robot == r, :);
        for k = 1:numel(evs)
            lim = min(nxt(k), evs(k) + 30);
            S = Tr(Tr.t >= evs(k) & Tr.t < lim, :);
            ok = S.hasTrack & S.err < 1 & S.inRegion95; nw = max(1, round(1/params.evalDt));
            % reacquired = first time good for 1 s consecutively (10 samples at 0.1)
            n = numel(ok); g = false(n, 1);
            for i = 1:n - nw + 1, g(i) = all(ok(i:i + nw - 1)); end
            i = find(g, 1);
            if isempty(i), tt(end+1) = lim - evs(k); M.reacqFail = M.reacqFail + 1; %#ok<AGROW>
            else, tt(end+1) = S.t(i) - evs(k); end %#ok<AGROW>
            W = Tr(Tr.t >= evs(k) & Tr.t < evs(k) + win, :);
            pe = [pe; W.err(W.hasTrack)]; %#ok<AGROW>
            pi_ = [pi_; W.inRegion95]; %#ok<AGROW>
        end
    end
    M.reacq = median(tt); M.reacqMean = mean(tt); M.postErr = median(pe); M.postIn = mean(pi_);
end
M.T = T; M.Tall = Tall;
end
