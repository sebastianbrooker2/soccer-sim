% Three-way comparison on real01 (packetSource='yolo'): NUbots faithful baseline, own-only SRIF, joint SRIF
% (+ tuned baseline for reference). Same packets, same ground truth, same 0.2 s grid. Writes results.csv,
% calib.csv, purity.csv, compare3.mat in this folder.  Run from anywhere (MATLAB -batch "run('.../run_compare3.m')").
here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
cd(root); addpath(genpath(root));
params = evalParams();
params.baseline.mode = 'faithful';
params.evalDt = 0.2; params.maxAreaForScore = 3;
gt = loadGroundTruth(fullfile(root,'eval','gt','real01.csv'), 0.05);
rng(1);
packets = makePacketsYolo(gt, params, fullfile(root,'recordings','real01','yolo'));
numObs = numel(params.observers);
delivered = rand(numObs, numel(packets)) >= params.linkDropProb;   % identical draw to tools/viz/export_tracks.m
rid = arrayfun(@(p) p.RobotID, packets);
fprintf('%d packets, delivered frac %.3f\n', numel(packets), mean(delivered(:)));

C = struct();
tic; C.nubots = runBaselineTracker(packets, params); fprintf('nubots %.0fs\n', toc);
pt = params; pt.baseline.mode = 'tuned';
tic; C.tuned = runBaselineTracker(packets, pt); fprintf('tuned %.0fs\n', toc);
tic; C.joint = runSharedTracker(packets, delivered, params); fprintf('joint %.0fs\n', toc);
own = cell(1, numObs);
for r = 1:numObs
    dm = false(numObs, numel(packets)); dm(r, rid == r) = true;
    tic; sr = runSharedTracker(packets, dm, params); fprintf('own r%d %.0fs\n', r, toc);
    own{r} = sr(r);
end
C.own = [own{:}];

C.raw = runRawMeasurement(packets, params);   % 4th behaviour: no tracking (eval/rawmeas)
names = {'nubots','own','joint','raw','tuned'}; labels = {'NUbots (faithful)','Own SRIF','Joint SRIF','Raw measurement','Tuned baseline'};
T = struct(); fcs = struct();
for i = 1:numel(names)
    n = names{i};
    if strcmp(n, 'raw'), fn = @(c,t,cl) rawEstimates(c,t,cl,params); fcs.(n) = C.(n);
    elseif any(strcmp(n, {'nubots','tuned'})), fn = @(c,t,cl) baselineEstimates(c,t,cl); fcs.(n) = C.(n);
    else, fn = @fastEstimates; fcs.(n) = fastCopies(C.(n)); end
    T.(n) = evaluateMatch(gt, fcs.(n), fn, params, n);
end

%% metrics table
nTrue = struct('ball', 1, 'opponent', 3);
rows = {};
for i = 1:numel(names)
    for cl = {'ball','opponent'}
        for sc = 0:numObs
            S = T.(names{i}); [g_, ~] = findgroups(S.robot, S.t, S.class); sh_ = splitapply(@sum, double(S.hasTrack), g_); S.falseTracks = S.nTracks - sh_(g_);
            S = S(S.class == cl{1}, :);
            if sc > 0, S = S(S.robot == sc, :); end
            h = S.hasTrack; ne = S.nees(h & isfinite(S.nees));
            w = S.withinR;
            nlive = S.nTracks;   % per sample (repeated per target for opponents)
            rows(end+1,:) = {labels{i}, cl{1}, iff(sc==0,'all',sprintf('robot%d',sc)), height(S), mean(h), ...
                mean(S.inRegion95), mean(S.inRegion95(h)), mean(S.inRegion95Capped), mean(S.inRegion95Capped(h)), ...
                mean(w(:,2)), mean(w(h,2)), sqrt(mean(S.err(h).^2)), median(S.err(h)), median(S.area, 'omitnan'), ...
                mean(ne), median(ne), mean(nlive), mean(nlive) - nTrue.(cl{1}), mean(S.falseTracks)}; %#ok<AGROW>
        end
    end
end
vn = {'tracker','class','scope','n','availability','inRegion95','inRegion95_cond','inRegion95_capped','inRegion95_capped_cond', ...
      'within05','within05_cond','rmse','median_err','median_area95','mean_nees_cond','median_nees_cond','mean_live_tracks','live_minus_true','false_tracks_per_sample'};
R = cell2table(rows, 'VariableNames', vn);
writetable(R, fullfile(here, 'results.csv'));
disp(R(R.scope == "all", :));

%% ghosts / duplicates / identity swaps on a 0.5 s grid (truth used for evaluation only)
tG = (gt.t(1)+5:0.5:gt.t(end)).'; opp = find(gt.team == 2);
G = {};
for i = 1:numel(names)
    n = names{i}; copies = fcs.(n); if strcmp(n, 'raw'), continue, end
    if any(strcmp(n, {'nubots','tuned'})), fn = @(c,t,cl) baselineEstimates(c,t,cl); else, fn = @fastEstimates; end
    for r = 1:numObs
        acc = struct('gB',0,'dB',0,'gR',0,'dR',0,'swap',0); lastLab = containers.Map('KeyType','char','ValueType','double');
        for t = tG.'
            k = find(gt.t >= t, 1); pb = gt.ball(k,:).'; TP = squeeze(gt.pos(k,:,opp));
            eb = fn(copies(r), t, 'ball'); er = fn(copies(r), t, 'robot');
            if ~isempty(eb)
                d = sqrt(sum((reshape([eb.mean],2,[]) - pb).^2, 1)); acc.gB = acc.gB + sum(d > 1.5); acc.dB = acc.dB + max(sum(d <= 1.5) - 1, 0);
            end
            if ~isempty(er)
                M = reshape([er.mean],2,[]); D = sqrt((M(1,:).' - TP(1,:)).^2 + (M(2,:).' - TP(2,:)).^2);
                [dm_, jn] = min(D, [], 2); acc.gR = acc.gR + sum(dm_ > 1.5);
                for q = 1:numel(opp), acc.dR = acc.dR + max(sum(jn == q & dm_ <= 1.5) - 1, 0); end
                for j = 1:numel(er)
                    if dm_(j) > 1.5, continue, end
                    nm = er(j).name;
                    if isKey(lastLab, nm) && lastLab(nm) ~= jn(j), acc.swap = acc.swap + 1; end
                    lastLab(nm) = jn(j);
                end
            end
        end
        nT = numel(tG);
        G(end+1,:) = {labels{i}, r, acc.gB/nT, acc.dB/nT, acc.gR/nT, acc.dR/nT, acc.swap}; %#ok<AGROW>
    end
end
Gt = cell2table(G, 'VariableNames', {'tracker','robot','ghost_ball_per_sample','dup_ball_per_sample','ghost_robot_per_sample','dup_robot_per_sample','robot_track_identity_swaps'});
writetable(Gt, fullfile(here, 'ghosts.csv')); disp(Gt);

%% association purity of the SRIF copies (event log with TrueLabel; diagnostics only)
Pu = {};
for src = {'own','joint'}
    cp = C.(src{1});
    for r = 1:numObs
        for cl = {'ball','robot'}
            Pu(end+1,:) = [{src{1}, r, cl{1}}, purity(cp(r), cl{1})]; %#ok<AGROW>
        end
    end
end
Put = cell2table(Pu, 'VariableNames', {'tracker','robot','class','n_fusions','purity','frac_false_positive_dets','n_spawns','n_dup_spawns','n_tracks_dominated_by_fp','label_changes_within_track','merge_n','merge_correct'});
writetable(Put, fullfile(here, 'purity.csv')); disp(Put);

%% per-observer measurement calibration vs truth
K = calibrate(packets, gt, params);
writetable(K, fullfile(here, 'calib.csv')); disp(K);

save(fullfile(here, 'compare3.mat'), 'T', 'R', 'Gt', 'Put', 'K', 'params', 'delivered', '-v7');
% logs for the window analysis
LOG = struct(); for n = {'own','joint'}, for r = 1:numObs, LOG.(n{1}){r} = C.(n{1})(r).log; end, end
save(fullfile(here, 'logs.mat'), 'LOG', '-v7');
fprintf('done\n');

%% ---------------------------------------------------------------- helpers
function v = iff(c, a, b), if c, v = a; else, v = b; end, end

function out = purity(copy, cls)
tr = copy.tracks; nm = {tr.Name}; isC = strcmp({tr.Class}, cls); mine = nm(isC);
ev = copy.log; ev = ev(ismember({ev.name}, mine));
sp = ev(strcmp({ev.type}, 'spawn')); as = ev(strcmp({ev.type}, 'assoc')); mg = ev(strcmp({ev.type}, 'merge'));
lab = [{sp.detLabel}, {as.detLabel}]; trk = [{sp.name}, {as.name}]; tt = [sp.t, as.t];
[~, o] = sort(tt); lab = lab(o); trk = trk(o);
ok = 0; N = numel(lab); chg = 0; nfpTrk = 0;
for u = unique(trk)
    L = lab(strcmp(trk, u{1})); [uu, ~, ic] = unique(L); cnt = accumarray(ic(:), 1); [~, im] = max(cnt); dom = uu{im};
    ok = ok + sum(strcmp(L, dom)); if strcmp(dom, 'fp'), nfpTrk = nfpTrk + 1; end
    Lq = L(~strcmp(L, 'fp')); chg = chg + sum(~strcmp(Lq(1:end-1), Lq(2:end)));
end
out = {N, ok/max(N,1), mean(strcmp(lab, 'fp')), numel(sp), sum(strcmp({sp.note}, 'duplicate')), nfpTrk, chg, numel(mg), sum([mg.flag])};
end

function K = calibrate(packets, gt, params)
% z-scores of every detection vs truth using the packet's stated sigmas (incl. observer pose cov / heading var),
% as noiseDensity would build them at the TRUE target position: tang = DetNoiseStd^2 + HeadVar + t'Ct/r^2,
% range = RangeStd(det)^2 + u'Cu. infl = factor the filter applies to ObserverCovariance (robot x obsCovInflate).
tp = params.tracker; N = numel(packets);
rowsK = {};
rec = struct('obs', [], 'et', [], 'cls', {{}}, 'ez', [], 'er', [], 'zt', [], 'zr', [], 'zt_i', [], 'zr_i', [], 'rng', [], 'bias', zeros(0,2), 'tt', []);
Nt = numel(gt.t);
for p = 1:N
    pk = packets(p); kk = min(max(round((pk.time - gt.t(1))/(gt.t(2)-gt.t(1))) + 1, 1), Nt);
    for d = 1:numel(pk.detections)
        dt = pk.detections(d); lab = dt.TrueLabel;
        if ~(strcmp(lab, 'ball') || strncmp(lab, 'opp', 3)), continue, end
        D = dt.TrueDelta(:); if any(isnan(D)), continue, end
        y = dt.y(:); u = y(1:2)/norm(y(1:2)); uT = D/norm(D); tv = [-uT(2); uT(1)];
        et = tv.'*(u - uT) ; er = y(3) - norm(D);   % tangential unit-vector error ~ bearing error (rad); range error (m)
        rs = pk.RangeNoiseStd; if isfield(dt.LastDetection,'RangeStd') && isfinite(dt.LastDetection.RangeStd), rs = dt.LastDetection.RangeStd; end
        isB = strcmp(dt.Class,'ball'); infl = tp.obsCovInflateBall; if ~isB, infl = tp.obsCovInflate; end
        Cc = pk.ObserverCovariance;
        s2t = @(f) pk.DetectionNoiseStd^2 + pk.ObserverHeadingVariance + (tv.'*(f*Cc)*tv)/norm(D)^2;
        s2r = @(f) rs^2 + uT.'*(f*Cc)*uT;
        rec.obs(end+1) = pk.RobotID; rec.cls{end+1} = iff(isB,'ball','opponent'); rec.et(end+1) = et; rec.er(end+1) = er; %#ok<AGROW>
        rec.zt(end+1) = et/sqrt(s2t(1)); rec.zr(end+1) = er/sqrt(s2r(1));
        rec.zt_i(end+1) = et/sqrt(s2t(infl)); rec.zr_i(end+1) = er/sqrt(s2r(infl)); rec.rng(end+1) = norm(D);
        rec.tt(end+1) = pk.time;
        % believed-minus-true observer position (diagnostic of the AR(1) bias)
        rec.bias(end+1,:) = (pk.ObserverPosition(:) - squeeze(gt.pos(kk,:,pk.RobotID)).').';
    end
end
cls = rec.cls(:); obs = rec.obs(:); K = table();
for c = {'ball','opponent'}
    for o = [0 1 2 3]
        for rb = 1:3
            edges = [0 4 7 30];
            m = strcmp(cls, c{1}) & (o == 0 | obs == o) & rec.rng(:) >= edges(rb) & rec.rng(:) < edges(rb+1);
            if sum(m) < 5, continue, end
            zt = rec.zt(m); zr = rec.zr(m); zti = rec.zt_i(m); zri = rec.zr_i(m);
            rwo = @(x) 1.4826*median(abs(x - median(x)));
            row = table(string(c{1}), o, string(sprintf('%d-%s m', edges(rb), iff(rb==3,'inf',num2str(edges(rb+1))))), sum(m), ...
                mean(rec.rng(m)), mean(zt), std(zt), rwo(zt), mean(abs(zt) > 2), mean(zr), std(zr), rwo(zr), mean(abs(zr) > 2), ...
                mean(zt.^2 + zr.^2), mean(zti.^2 + zri.^2), median(zt.^2 + zr.^2), ...
                sqrt(mean(rec.et(m).^2)), sqrt(mean(rec.er(m).^2)), ...
                'VariableNames', {'class','observer_0all','range_bin','n','mean_range','z_tang_mean','z_tang_std','z_tang_robust_std','z_tang_frac_gt2', ...
                'z_range_mean','z_range_std','z_range_robust_std','z_range_frac_gt2','nees2_stated','nees2_inflated','nees2_median_stated','rms_bearing_rad','rms_range_m'});
            K = [K; row]; %#ok<AGROW>
        end
    end
end
% observer pose-bias statistics (believed - true), per observer
for o = 1:3
    m = rec.obs(:) == o; b = rec.bias(m,:); tt = rec.tt(m);
    fprintf('observer %d pose bias: std x %.3f y %.3f m (configured %.3f), mean %.3f %.3f\n', o, std(b(:,1)), std(b(:,2)), sqrt(pk_posvar(packets)), mean(b(:,1)), mean(b(:,2)));
end
end
function s = pk_posvar(packets), s = packets(1).ObserverCovariance(1,1); end
