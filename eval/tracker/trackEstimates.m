function est = trackEstimates(copy, tEval, class)
%TRACKESTIMATES Live tracks of one copy at time TEVAL.
%   est = trackEstimates(copy, tEval [, class]) -> struct array with fields
%     name, mean (2x1 position), cov (2x2 position covariance), class.
%   CLASS: 'ball' | 'robot' | omitted/'' (all).
%
%   Minimal track struct read from copy.tracks(k) (works for shared AND
%   baseline copies):
%     .Name (or .name), .Class (or .class)  char
%     .alive (logical), .deletedAt (time, NaN if alive), .SpawnTime
%     .queue(i).time      history time stamps (nondecreasing)
%     .queue(i).system.density  with mean() (>=2x1, position first) and cov()
%     .queue(i).system.time     (optional) and .system.predict(t)
%                               (optional; if present the estimate is
%                               predicted forward from the latest entry
%                               with time <= tEval to tEval; else used as is)
%   A track is live at tEval if SpawnTime <= tEval and (alive or
%   deletedAt > tEval) and it has a queue entry with time <= tEval.
if nargin < 3, class = ''; end
est = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
tr = copy.tracks;
for k = 1:numel(tr)
    T = tr(k);
    cl = getf(T, {'Class', 'class'}, '');
    if ~isempty(class) && ~strcmp(cl, class), continue, end
    if T.SpawnTime > tEval, continue, end
    if ~T.alive && ~(T.deletedAt > tEval), continue, end
    q = T.queue;
    if isempty(q), continue, end
    qt = [q.time];
    i = find(qt <= tEval, 1, 'last');
    if isempty(i), continue, end
    sys = q(i).system;
    if isprop(sys, 'time') || isfield(sys, 'time')
        dt = tEval - sys.time;
    else
        dt = 0;
    end
    if dt > 0 && ismethod(sys, 'predict')
        sys = sys.predict(tEval);
    end
    m = sys.density.mean();
    P = sys.density.cov();
    est(end + 1) = struct('name', getf(T, {'Name', 'name'}, sprintf('trk%d', k)), ...
        'mean', m(1:2), 'cov', (P(1:2, 1:2) + P(1:2, 1:2).')/2, 'class', cl); %#ok<AGROW>
end
end

function v = getf(s, names, default)
v = default;
for n = 1:numel(names)
    if isfield(s, names{n}), v = s.(names{n}); return, end
end
end
