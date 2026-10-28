function M = oppMetrics(Tall, E)
%OPPMETRICS Opponent metrics from the evaluateMatch table; E = robot events table
%   (type=="robot", columns t,id) for reacquire time of the teleported opponent.
T = Tall(Tall.class == "opponent", :);
M.availability = mean(T.hasTrack);
M.inRegion95 = mean(T.inRegion95);
M.inRegion95Cond = mean(T.inRegion95(T.hasTrack));
M.rmse = sqrt(mean(T.err(T.hasTrack).^2));
M.medErr = median(T.err(T.hasTrack));
M.falseTracks = mean(T.falseTracks);
M.areaMedian = median(T.area, 'omitnan');
tt = []; pe = [];
E = E(E.type == "robot" & E.id >= 4, :);
for i = 1:height(E)
    for r = unique(T.robot)'
        S = T(T.robot == r & T.target == E.id(i) & T.t >= E.t(i) & T.t < E.t(i) + 30, :);
        ok = S.hasTrack & S.err < 1 & S.inRegion95; n = numel(ok); g = false(n, 1); nw = max(1, round(1/params_dt(Tall)));
        for j = 1:n - nw + 1, g(j) = all(ok(j:j+nw-1)); end
        j = find(g, 1);
        if isempty(j), tt(end+1) = 30; else, tt(end+1) = S.t(j) - E.t(i); end %#ok<AGROW>
        W = S(S.t < E.t(i) + 5, :); pe = [pe; W.inRegion95]; %#ok<AGROW>
    end
end
M.nEv = height(E); M.reacq = median(tt); M.reacqMean = mean(tt); M.postIn = mean(pe);
end

function d = params_dt(T), u = unique(T.t(1:min(end,2000))); d = median(diff(u)); end
