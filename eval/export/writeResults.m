function writeResults(results, outDir)
%WRITERESULTS Write per_match.csv, summary.csv, timeseries_match<k>.csv, results.mat
%   and the aggregated plot-data CSVs used by the pgfplots figures.
%   results fields:
%     perMatch   table: match,tracker,class,robot,inRegion95,within_*,rmse,availability,region_area_median
%     summary    (optional) table from aggregateMatches; computed if missing
%     timeseries cell{k}: concatenated evaluateMatch tables (all trackers) for match k
%     params     (optional) evalParams; uses params.tsStride (keep every n-th eval time, default 1)
%   Extra CSVs: inregion_time.csv (t + <tracker>_<class> columns, mean over
%   matches/robots/targets in tsBinSec bins) and err_cdf.csv (x + same columns).
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~isfield(results, 'summary') || isempty(results.summary)
    results.summary = aggregateMatches(results.perMatch);
end
params = struct(); if isfield(results, 'params'), params = results.params; end
stride = 1; if isfield(params, 'tsStride'), stride = max(1, params.tsStride); end

pm = results.perMatch;
if ismember('match', pm.Properties.VariableNames), pm = movevars(pm, 'match', 'Before', 1); end
writetable(pm, fullfile(outDir, 'per_match.csv'));
writetable(results.summary, fullfile(outDir, 'summary.csv'));

ts = {};
if isfield(results, 'timeseries'), ts = results.timeseries; end
for k = 1:numel(ts)
    T = ts{k}; if isempty(T), continue; end
    if stride > 1
        ut = unique(T.t); keep = ismember(T.t, ut(1:stride:end)); T = T(keep,:);
    end
    O = table(T.t, T.tracker, T.class, T.robot, T.target, T.err, ...
        double(T.inRegion95), double(T.hasTrack), 'VariableNames', ...
        {'t','tracker','class','robot','target','err','inRegion95','hasTrack'});
    writetable(O, fullfile(outDir, sprintf('timeseries_match%d.csv', k)));
end
writePlotData(ts, outDir, params);
save(fullfile(outDir, 'results.mat'), 'results');
end

function writePlotData(ts, outDir, params)
ts = ts(~cellfun(@isempty, ts));
if isempty(ts), return; end
A = vertcat(ts{:});
binSec = 10; if isfield(params, 'tsBinSec'), binSec = params.tsBinSec; end
trk = unique(A.tracker, 'stable'); cls = unique(A.class(A.class ~= "teammate"), 'stable');
tb = floor(A.t / binSec) * binSec; edges = unique(tb);
names = {}; Y = nan(numel(edges), 0);
xg = linspace(0, 2, 201)'; C = nan(numel(xg), 0);
for a = 1:numel(trk)
  for b = 1:numel(cls)
    m = A.tracker == trk(a) & A.class == cls(b);
    if ~any(m), continue; end
    names{end+1} = colName(trk(a), cls(b)); %#ok<AGROW>
    [ub, ~, gi] = unique(tb(m));
    y = accumarray(gi, double(A.inRegion95(m)), [], @mean);
    col = nan(numel(edges),1); col(ismember(edges, ub)) = y;
    Y(:,end+1) = col; %#ok<AGROW>
    e = A.err(m); e(isnan(e)) = Inf;     % misses never fall below any radius
    C(:,end+1) = arrayfun(@(x) mean(e <= x), xg); %#ok<AGROW>
  end
end
writetable(array2table([edges Y], 'VariableNames', [{'t'} names]), fullfile(outDir, 'inregion_time.csv'));
writetable(array2table([xg C], 'VariableNames', [{'x'} names]), fullfile(outDir, 'err_cdf.csv'));
end
