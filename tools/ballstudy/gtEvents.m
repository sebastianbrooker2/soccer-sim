function E = gtEvents(gt, opts)
%GTEVENTS Combined teleport / out-of-play events from ground truth.
%   STAND-IN for a referee / GameController signal (external information,
%   never used by the pure-tracking options A-C).
%   E: table  t,type('ball'|'robot'),id,from_x,from_y,to_x,to_y,kind
%   robot (kind 'teleport'): position change > robotV (default 6 m/s, walking <1 m/s)
%     between samples; consecutive jumpy samples merged into one event;
%     t = time of the last normal sample, from/to = positions before/after the run.
%     (real01: these coincide with 'active'=false in the replay jsonl.)
%   ball: kind 'jump' (speed > ballJumpV, default 9 m/s), 'exit' (leaves
%     |x|>7 or |y|>4.5 for >= 1.5 s; from = last in-field pos, to = pos at exit) and
%     'restart' (end of that stretch; from = pos there, to = pos 1 s later).
if nargin < 2, opts = struct(); end
robotV = fo(opts, 'robotV', 6); ballV = fo(opts, 'ballJumpV', 9);
t = gt.t; rows = {};
for k = 1:6
    p = squeeze(gt.pos(:, :, k));
    v = [0; hypot(diff(p(:,1)), diff(p(:,2))) ./ diff(t)];
    j = v > robotV; d = diff([0; j; 0]); s = find(d == 1); e = find(d == -1) - 1;
    for i = 1:numel(s)
        rows(end+1, :) = {t(s(i)-1), 'robot', k, p(s(i)-1,1), p(s(i)-1,2), p(e(i),1), p(e(i),2), 'teleport'}; %#ok<AGROW>
    end
end
b = gt.ball; vb = [0; hypot(diff(b(:,1)), diff(b(:,2))) ./ diff(t)];
j = vb > ballV; d = diff([0; j; 0]); s = find(d == 1); e = find(d == -1) - 1;
for i = 1:numel(s)
    rows(end+1, :) = {t(s(i)-1), 'ball', 0, b(s(i)-1,1), b(s(i)-1,2), b(e(i),1), b(e(i),2), 'jump'}; %#ok<AGROW>
end
ev = gtBallEvents(gt);
for i = 1:size(ev.stretch, 1)
    ie = find(t >= ev.stretch(i, 1), 1); ir = find(t >= ev.stretch(i, 2), 1); ip = min(ir + 20, numel(t));
    rows(end+1, :) = {t(ie), 'ball', 0, b(max(ie-1,1),1), b(max(ie-1,1),2), b(ie,1), b(ie,2), 'exit'}; %#ok<AGROW>
    rows(end+1, :) = {t(ir), 'ball', 0, b(ir,1), b(ir,2), b(ip,1), b(ip,2), 'restart'}; %#ok<AGROW>
end
E = cell2table(rows, 'VariableNames', {'t','type','id','from_x','from_y','to_x','to_y','kind'});
E.type = string(E.type); E.kind = string(E.kind);
E = sortrows(E, 't');
end
function v = fo(o, f, d), if isfield(o, f), v = o.(f); else, v = d; end, end
