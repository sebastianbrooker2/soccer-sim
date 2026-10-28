function D = trackerDiagnostics(gt, copies, params)
%TRACKERDIAGNOSTICS Diagnostics of a shared-tracker run (uses truth: EVALUATION ONLY).
%   D = trackerDiagnostics(gt, copies, params). Per-copy averages in D:
%     nLive      mean # live robot tracks (compare nTrue = # robot targets in packets)
%     spawnPerMin robot-track spawns per sim-minute
%     dupFrac    fraction of robot spawns logged as 'duplicate' (a live track existed)
%     assocOK    fraction of robot assoc events with the right label
%     mergeOK    fraction of robot merges that were correct (nMerge also given)
%     sd1        median position sd (m) of new robot tracks after first fusion
%     nees       mean NEES of nearest-track estimate per opponent (nominal 2), nees95 = 95th pct
%     in95c      inRegion95 conditional on a track within 3 m
%     avail      fraction of time/targets with a track within 3 m
%     errMed     median error (m) of nearest track
%     ghost      mean # live robot tracks > 1.5 m from every true robot
%     dup        mean # extra tracks (beyond one per true robot) within 1.5 m of a true robot
%     minSep     (info) median over time of the smallest true inter-robot distance
nTrue = 3 + ~getOr(params, 'dropTeammateDetections', true) * 2;
tE = (gt.t(1) + 5):0.25:gt.t(end);
opp = find(gt.team == 2);
chi = 5.9915;
nC = numel(copies);
D = struct('nLive', 0, 'spawnPerMin', 0, 'dupFrac', 0, 'assocOK', 0, 'mergeOK', 0, 'nMerge', 0, ...
    'sd1', 0, 'nees', 0, 'nees95', 0, 'in95c', 0, 'avail', 0, 'errMed', 0, 'ghost', 0, 'dup', 0, 'ballIn95', 0, 'ballAvail', 0, 'ballErr', 0, 'ballNees', 0, 'nTrue', nTrue);
acc = zeros(nC, 18);
for r = 1:nC
    ev = copies(r).log;
    isRob = false(1, numel(ev));
    tr = copies(r).tracks;
    names = {tr.Name}; cls = {tr.Class};
    for i = 1:numel(ev), isRob(i) = any(strcmp(ev(i).name, names)) && strcmp(cls{find(strcmp(names, ev(i).name), 1)}, 'robot'); end
    ev = ev(isRob);
    sp = ev(strcmp({ev.type}, 'spawn'));
    as = ev(strcmp({ev.type}, 'assoc'));
    mg = ev(strcmp({ev.type}, 'merge'));
    dur = gt.t(end) - gt.t(1);
    bn = []; bh = []; be = [];
    for i = 1:numel(tE)
        eb = trackEstimates(copies(r), tE(i), 'ball'); k = find(gt.t >= tE(i), 1); pb = gt.ball(k, :).';
        if isempty(eb), bh(end+1) = false; continue, end %#ok<AGROW>
        Mb = reshape([eb.mean], 2, []); [e, j] = min(sqrt(sum((Mb - pb).^2, 1)));
        bh(end+1) = true; be(end+1) = e; dd_ = Mb(:, j) - pb; bn(end+1) = dd_.' * (eb(j).cov \ dd_); %#ok<AGROW>
    end
    gh = zeros(size(tE)); du = zeros(size(tE)); nL = zeros(size(tE)); nees = []; inr = []; has = []; er = [];
    for i = 1:numel(tE)
        est = trackEstimates(copies(r), tE(i), 'robot');
        nL(i) = numel(est);
        k = find(gt.t >= tE(i), 1);
        tr_ids = opp(:)';
        if ~getOr(params, 'dropTeammateDetections', true), tr_ids = [find(gt.team == 1 & (1:numel(gt.team)) ~= r), tr_ids]; end
        TP = squeeze(gt.pos(k, :, tr_ids));
        if ~isempty(est)
            Me = reshape([est.mean], 2, []);
            Dm = sqrt((Me(1,:).' - TP(1,:)).^2 + (Me(2,:).' - TP(2,:)).^2);   % nTrk x nTrue
            [dmin, jn] = min(Dm, [], 2);
            gh(i) = sum(dmin > 1.5);
            for q = 1:numel(tr_ids), du(i) = du(i) + max(sum(jn == q & dmin <= 1.5) - 1, 0); end
        end
        for o = opp(:)'
            p = squeeze(gt.pos(k, :, o)).';
            if isempty(est), has(end+1) = false; continue, end %#ok<AGROW>
            M = reshape([est.mean], 2, []);
            [e, j] = min(sqrt(sum((M - p).^2, 1)));
            if e > 3, has(end+1) = false; continue, end %#ok<AGROW>
            has(end+1) = true; %#ok<AGROW>
            d = M(:, j) - p;
            nn = d.' * (est(j).cov \ d);
            nees(end+1) = nn; inr(end+1) = nn <= chi; er(end+1) = e; %#ok<AGROW>
        end
    end
    acc(r, :) = [mean(nL), numel(sp)/dur*60, mean(strcmp({sp.note}, 'duplicate')), mean([as.flag]), ...
        mean([mg.flag]), numel(mg), median([sp.life]), mean(nees), prctile(nees, 95), mean(inr), mean(has), median(er), mean(gh), mean(du), mean(bn <= chi), mean(bh), median(be), mean(bn)];
end
m = mean(acc, 1, 'omitnan');
f = {'nLive', 'spawnPerMin', 'dupFrac', 'assocOK', 'mergeOK', 'nMerge', 'sd1', 'nees', 'nees95', 'in95c', 'avail', 'errMed', 'ghost', 'dup', 'ballIn95', 'ballAvail', 'ballErr', 'ballNees'};
for i = 1:numel(f), D.(f{i}) = m(i); end
end

function v = getOr(s, n, d)
if isfield(s, n), v = s.(n); else, v = d; end
end
