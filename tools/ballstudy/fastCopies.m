function fc = fastCopies(copies)
%FASTCOPIES Precompute queue time vectors / systems so fastEstimates is O(live tracks).
%   Same results as trackEstimates (also returns lastUpdate); evaluation speed only.
fc = copies;
for r = 1:numel(copies)
    tr = copies(r).tracks; out = struct('Name', {}, 'Class', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'qt', {}, 'qs', {});
    for k = 1:numel(tr)
        q = tr(k).queue;
        if isempty(q), continue, end
        qs = cell(1, numel(q)); for i = 1:numel(q), qs{i} = q(i).system; end
        out(end+1) = struct('Name', tr(k).Name, 'Class', tr(k).Class, 'alive', tr(k).alive, 'deletedAt', tr(k).deletedAt, ...
            'SpawnTime', tr(k).SpawnTime, 'qt', [q.time], 'qs', {qs}); %#ok<AGROW>
    end
    fc(r).tracks = out;
    fc(r).spawn = [out.SpawnTime]; fc(r).alive = [out.alive];
    dd = [out.deletedAt]; dd(isnan(dd)) = -inf; fc(r).del = dd;   % NaN deletedAt: dead tracks never live after spawn
    fc(r).isBall = strcmp({out.Class}, 'ball'); fc(r).qt0 = arrayfun(@(o) o.qt(1), out);
end
end
