function tl = trackLastUpdate(copy, E, tEval)
%TRACKLASTUPDATE Time of latest stored state (<= tEval) of each estimate in E.
%   Uses E(i).lastUpdate if the estimator provides it; otherwise looks the
%   track up by name in copy.tracks (shared: queue(i).time, baseline: t).
%   NaN if it cannot be determined.
tl = nan(1, numel(E));
if isempty(E), return; end
if isfield(E, 'lastUpdate')
    for i = 1:numel(E), if ~isempty(E(i).lastUpdate), tl(i) = E(i).lastUpdate; end, end
    if ~any(isnan(tl)), return; end
end
if ~isfield(copy, 'tracks'), return; end
tr = copy.tracks;
for i = 1:numel(E)
    if ~isnan(tl(i)), continue; end
    for k = 1:numel(tr)
        nm = '';
        if isfield(tr, 'Name'), nm = tr(k).Name; elseif isfield(tr, 'name'), nm = tr(k).name; end
        if ~isequal(nm, E(i).name), continue; end
        if isfield(tr, 'queue') && ~isempty(tr(k).queue)
            qt = [tr(k).queue.time]; j = find(qt <= tEval, 1, 'last');
            if ~isempty(j), tl(i) = qt(j); end
        elseif isfield(tr, 't') && ~isempty(tr(k).t)
            j = find(tr(k).t <= tEval, 1, 'last');
            if ~isempty(j), tl(i) = tr(k).t(j); end
        end
        break;
    end
end
end
