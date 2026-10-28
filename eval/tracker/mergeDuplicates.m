function [tracks, evs] = mergeDuplicates(tracks, t, params)
%MERGEDUPLICATES Drop duplicate live tracks: two same-class tracks whose
%   position estimates agree within the merge gate (position difference
%   against the SUM of their covariances) are the same object tracked
%   twice. The one with fewer fused detections is deleted (the younger on
%   a tie).

evs = makeEvent('x', 0, '', '', '', 0, 0, 0, false, '', 0);
evs(1) = [];

idx = find([tracks.alive]);
n   = numel(idx);
mu  = zeros(2, n);
P   = zeros(2, 2, n);
for a = 1:n
    m = tracks(idx(a)).system.density.mean();
    C = tracks(idx(a)).system.density.cov();
    mu(:, a)    = m(1:2);
    P(:, :, a) = C(1:2, 1:2);
end

for a = 1:n - 1
    for b = a + 1:n
        i = idx(a);
        j = idx(b);
        if ~tracks(i).alive || ~tracks(j).alive || ~strcmp(tracks(i).Class, tracks(j).Class) ...
                || ismember(j, tracks(i).distinct)
            continue
        end
        if sqrt(max(eig(P(:, :, a)))) > params.mergeMaxStd || sqrt(max(eig(P(:, :, b)))) > params.mergeMaxStd
            continue % too vague to call it a duplicate
        end
        if numel(tracks(i).queue) < params.mergeMinFusions || numel(tracks(j).queue) < params.mergeMinFusions
            continue % a track that has barely been confirmed cannot yet be called a duplicate
        end
        dm = mu(:, a) - mu(:, b);
        d2 = dm.' * ((P(:, :, a) + P(:, :, b)) \ dm);
        if d2 > params.mergeThreshold
            continue
        end
        ni = numel(tracks(i).queue);
        nj = numel(tracks(j).queue);
        if ni > nj || (ni == nj && tracks(i).SpawnTime <= tracks(j).SpawnTime)
            keep = i; drop = j;
        else
            keep = j; drop = i;
        end
        tracks(drop).alive        = false;
        tracks(drop).deletedAt    = t;
        tracks(drop).DeleteReason = 'merged';
        evs = [evs, makeEvent('merge', t, tracks(drop).Name, tracks(drop).TrueLabel, tracks(keep).TrueLabel, ...
            d2, norm(dm), numel(tracks(drop).queue), strcmp(tracks(drop).TrueLabel, tracks(keep).TrueLabel), ...
            tracks(keep).Name, t - tracks(drop).SpawnTime)]; %#ok<AGROW>
    end
end
end
