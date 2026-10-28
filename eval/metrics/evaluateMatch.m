function T = evaluateMatch(gt, copies, estimatesFcn, params, trackerName)
%EVALUATEMATCH Per-sample accuracy of tracker estimates against ground truth.
%   T = evaluateMatch(gt, copies, estimatesFcn, params, trackerName)
%   gt: struct t (Nt x1), ball (Nt x2), pos (Nt x2 x6), team (1x6).
%   copies: struct array, one per observing robot r (robot id r).
%   estimatesFcn(copy, tEval, class) -> struct array (name, mean 2x1, cov 2x2,
%       class), class is 'ball' or 'robot'.
%   params: evalDt, radii, [evalTeammates=false], [confidence=0.95], [evalT0].
%   Output table, one row per (robot, tEval, target):
%     tracker, t, robot, class ('ball'|'opponent'|'teammate'), target (0..6),
%     err, inRegion95, withinR (nRadii logical matrix), hasTrack, area
%   Misses (no live track of the right class): hasTrack=false, err=NaN,
%   inRegion95=false, withinR=false, area=NaN.
if nargin < 5, trackerName = 'tracker'; end
conf = getf(params, 'confidence', 0.95);
chi2 = -2*log(1-conf);               % chi2inv(conf,2), no toolbox needed
radii = params.radii(:)'; nR = numel(radii);
evalTm = getf(params, 'evalTeammates', false);
t0 = getf(params, 'evalT0', gt.t(1));
tEval = (t0:params.evalDt:gt.t(end))';
nT = numel(tEval);
nObs = numel(copies);

oppIds = find(gt.team == 2);
tmIds  = find(gt.team == 1);
% truth at tEval (columns: target ordering [ball, opp..., teammates...])
targets = struct('id', 0, 'cls', 'ball', 'est', 'ball');
for k = oppIds(:)', targets(end+1) = struct('id', k, 'cls', 'opponent', 'est', 'robot'); end %#ok<AGROW>
if evalTm
    for k = tmIds(:)', targets(end+1) = struct('id', k, 'cls', 'teammate', 'est', 'robot'); end %#ok<AGROW>
end
nG = numel(targets);
truth = nan(nT, 2, nG);
for g = 1:nG
    if targets(g).id == 0, src = gt.ball; else, src = gt.pos(:,:,targets(g).id); end
    truth(:,:,g) = interp1(gt.t, src, tEval, 'linear', 'extrap');
end

matchMode = getf(params, 'matchMode', 'onetoone');
gate = getf(params, 'matchGate', 3);
maxArea = getf(params, 'maxAreaForScore', Inf);
maxAge = getf(params, 'recentAge', 1);
oneToOne = strcmp(matchMode, 'onetoone');

maxRows = nObs*nT*nG;
tr = zeros(maxRows,1); rb = zeros(maxRows,1); tg = zeros(maxRows,1);
cl = strings(maxRows,1);
err = nan(maxRows,1); inR = false(maxRows,1); wR = false(maxRows,nR);
has = false(maxRows,1); area = nan(maxRows,1);
nees = nan(maxRows,1); nTrk = nan(maxRows,1); nTgt = nan(maxRows,1);
areaRec = nan(maxRows,1); age = nan(maxRows,1);
n = 0;
for r = 1:nObs
    % true robot targets for the assignment (every robot except the observer)
    rIds = [oppIds(:); tmIds(:)]; rIds = rIds(rIds ~= r)';
    rTruth = nan(nT, 2, numel(rIds));
    for q = 1:numel(rIds), rTruth(:,:,q) = interp1(gt.t, gt.pos(:,:,rIds(q)), tEval, 'linear', 'extrap'); end
    for i = 1:nT
        te = tEval(i);
        estBall = estimatesFcn(copies(r), te, 'ball');
        estRob  = estimatesFcn(copies(r), te, 'robot');
        mB = collectMeans(estBall); mR = collectMeans(estRob);
        aB = lastAge(copies(r), estBall, te); aR = lastAge(copies(r), estRob, te);
        % per-class sample stats (all live tracks, regardless of matching)
        arB = areas(estBall, chi2); arR = areas(estRob, chi2);
        recB = median(arB(isnan(aB) | aB < maxAge), 'omitnan'); recR = median(arR(isnan(aR) | aR < maxAge), 'omitnan');
        % one-to-one assignment: track index per target (0 = none)
        asgB = 0; asgR = zeros(1, numel(rIds));
        if oneToOne
            asgB = assign(mB, reshape(truth(i,:,1), 1, 2), gate);
            asgR = assign(mR, reshape(permute(rTruth(i,:,:), [3 2 1]), [], 2), gate);
        end
        for g = 1:nG
            isBall = targets(g).id == 0;
            if targets(g).id == r && ~isBall, continue; end
            n = n + 1;
            tr(n) = te; rb(n) = r; tg(n) = targets(g).id; cl(n) = targets(g).cls;
            if isBall
                M = mB; E = estBall; nTrk(n) = numel(estBall); nTgt(n) = 1; areaRec(n) = recB; Ag = aB;
            else
                M = mR; E = estRob; nTrk(n) = numel(estRob); nTgt(n) = numel(rIds); areaRec(n) = recR; Ag = aR;
            end
            if isempty(M), continue; end
            p = squeeze(truth(i,:,g))';
            if oneToOne
                if isBall, j = asgB(1); else, j = asgR(rIds == targets(g).id); end
                if j == 0, continue; end
            else
                d = sqrt(sum((M - p).^2, 1));
                [~, j] = min(d);
            end
            e = norm(M(:,j) - p);
            P = E(j).cov; ev = M(:,j) - p;
            has(n) = true; err(n) = e; age(n) = Ag(j);
            wR(n,:) = e <= radii;
            dP = det(P);
            if dP > 0 && all(isfinite(P(:)))
                nees(n) = ev' * (P \ ev);
                inR(n) = nees(n) <= chi2;
                area(n) = pi*chi2*sqrt(dP);
            end
        end
    end
end
idx = 1:n;
inRc = inR(idx) & area(idx) < maxArea;
T = table(repmat(string(trackerName), n, 1), tr(idx), rb(idx), cl(idx), tg(idx), ...
    err(idx), inR(idx), wR(idx,:), has(idx), area(idx), ...
    nees(idx), nTrk(idx), nTgt(idx), nTrk(idx) - nTgt(idx), areaRec(idx), age(idx), inRc, ...
    'VariableNames', {'tracker','t','robot','class','target','err','inRegion95','withinR','hasTrack','area', ...
    'nees','nTracks','nTargets','falseTracks','areaRecent','age','inRegion95Capped'});
end

function j = assign(M, P, gate)
% Hungarian assignment of tracks (columns of M, 2xnT) to targets (rows of P, nG x 2)
% by Euclidean distance; pairs farther than gate stay unmatched. j(g) = track idx or 0.
nG = size(P, 1); j = zeros(1, nG);
if isempty(M), return; end
C = sqrt((P(:,1) - M(1,:)).^2 + (P(:,2) - M(2,:)).^2);   % nG x nT
a = matchpairs(C, gate/2);   % unmatched cost gate/2 per side => pair kept iff d < gate
for q = 1:size(a,1), j(a(q,1)) = a(q,2); end
end

function a = areas(E, chi2)
a = nan(1, numel(E));
for i = 1:numel(E)
    d = det(E(i).cov);
    if d > 0 && all(isfinite(E(i).cov(:))), a(i) = pi*chi2*sqrt(d); end
end
end

function a = lastAge(copy, E, te)
if isempty(E), a = zeros(1,0); return; end
a = te - trackLastUpdate(copy, E, te);
end

function M = collectMeans(E)
if isempty(E), M = zeros(2,0); return; end
M = reshape([E.mean], 2, []);
end

function v = getf(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
