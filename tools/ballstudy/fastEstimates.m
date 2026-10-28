function est = fastEstimates(copy, tEval, class)
%FASTESTIMATES trackEstimates equivalent on fastCopies output (adds lastUpdate).
est = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {}, 'lastUpdate', {});
if ~isfield(copy, 'spawn')   % old-style fastCopies output
    for T = copy.tracks
        if ~strcmp(T.Class, class) || T.SpawnTime > tEval || (~T.alive && ~(T.deletedAt > tEval)), continue, end
        i = find(T.qt <= tEval, 1, 'last'); if isempty(i), continue, end
        sys = T.qs{i}; if tEval - sys.time > 0, sys = sys.predict(tEval); end
        m = sys.density.mean(); P = sys.density.cov();
        est(end+1) = struct('name', T.Name, 'mean', m(1:2), 'cov', (P(1:2,1:2) + P(1:2,1:2).')/2, 'class', T.Class, 'lastUpdate', T.qt(i)); %#ok<AGROW>
    end
    return
end
isC = copy.isBall; if ~strcmp(class, 'ball'), isC = ~isC; end
cand = find(isC & copy.spawn <= tEval & (copy.alive | copy.del > tEval) & copy.qt0 <= tEval);
for k = cand
    T = copy.tracks(k);
    i = find(T.qt <= tEval, 1, 'last');
    sys = T.qs{i};
    if tEval - sys.time > 0, sys = sys.predict(tEval); end
    m = sys.density.mean(); P = sys.density.cov();
    est(end+1) = struct('name', T.Name, 'mean', m(1:2), 'cov', (P(1:2,1:2) + P(1:2,1:2).')/2, 'class', T.Class, 'lastUpdate', T.qt(i)); %#ok<AGROW>
end
end
