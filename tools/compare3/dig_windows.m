% Explain windows where joint SRIF is worse than own SRIF. Usage: set W = {class, t0, t1; ...} then run.
% Needs cop.mat (copies, packets, gt) written by the caller (path in COP). Prints truth/tracks/events per window.
if ~exist('COP','var'), COP = fullfile(tempdir,'cop.mat'); end
if ~exist('WL','var'), WL = {'ball',460,470; 'ball',720,730; 'opponent',430,440; 'opponent',580,590; 'opponent',190,200}; end
Z = load(COP); gt = Z.gt; fj = fastCopies(Z.joint); fo = fastCopies(Z.own);
for w = 1:size(WL,1)
    cl = WL{w,1}; t0 = WL{w,2}; t1 = WL{w,3}; tc = (t0+t1)/2;
    fprintf('\n=========== %s window [%g,%g] ===========\n', cl, t0, t1);
    k = find(gt.t >= tc, 1);
    if strcmp(cl,'ball'), tgt = gt.ball(k,:); fprintf('true ball at %.1f: (%.2f,%.2f)\n', tc, tgt); ecl = 'ball';
    else, for o = 4:6, fprintf('true opp%d at %.1f: (%.2f,%.2f)  ', o-3, tc, gt.pos(k,:,o)); end, fprintf('\n'); ecl = 'robot'; end
    for src = {'joint','own'}
        if strcmp(src{1},'joint'), F = fj; Cc = Z.joint; else, F = fo; Cc = Z.own; end
        for r = 1:3
            fprintf(' %s r%d:', src{1}, r);
            for t = [t0 tc t1]
                E = fastEstimates(F(r), t, ecl); s = '';
                for i = 1:numel(E)
                    if strcmp(cl,'ball'), d = norm(E(i).mean - tgt.'); else, d = min(arrayfun(@(o) norm(E(i).mean - squeeze(gt.pos(k,:,o)).'), 4:6)); end
                    s = [s sprintf(' %s(d=%.1f sd=%.2f)', E(i).name, d, sqrt(max(eig(E(i).cov))))]; %#ok<AGROW>
                end
                fprintf(' | t=%g:%s', t, s);
            end
            fprintf('\n');
            ev = Cc(r).log; ev = ev([ev.t] >= t0-3 & [ev.t] <= t1);
            ty = {ev.type}; trN = {Cc(r).tracks.Name}; trC = {Cc(r).tracks.Class};
            [~, ii] = ismember({ev.name}, trN); isc = ii > 0 & strcmp(trC(max(ii,1)), ecl);
            ev = ev(isc); ty = {ev.type};
            as = ev(strcmp(ty,'assoc')); wrong = as(~[as.flag]);
            fprintf('     events: spawn %d (dup %d) delete %d merge %d assoc %d (wrong label %d: %s)\n', sum(strcmp(ty,'spawn')), sum(strcmp({ev(strcmp(ty,'spawn')).note},'duplicate')), ...
                sum(strcmp(ty,'delete')), sum(strcmp(ty,'merge')), numel(as), numel(wrong), strjoin(unique(strcat({wrong.trackLabel}, '<-', {wrong.detLabel})), ','));
            dl = ev(strcmp(ty,'delete')); for q = 1:min(numel(dl),4), fprintf('       delete %s t=%.1f reason=%s life=%.1f\n', dl(q).name, dl(q).t, dl(q).note, dl(q).life); end
        end
    end
    % detections reaching the filters in the window, by robot and label
    P = Z.packets; m = [P.time] >= t0 & [P.time] <= t1; Pw = P(m);
    for r = 1:3
        lab = {}; for p = Pw([Pw.RobotID] == r), for d = p.detections, if strcmp(d.Class, ecl), lab{end+1} = d.TrueLabel; end, end, end %#ok<AGROW>
        [u,~,ic] = unique(lab); cn = accumarray(ic(:),1); str = ''; for q = 1:numel(u), str = [str sprintf('%s=%d ', u{q}, cn(q))]; end, fprintf(' robot%d class-%s detections in window: %s\n', r, ecl, str);
    end
end
