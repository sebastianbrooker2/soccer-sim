function S = summarizeMatch(T, params)
%SUMMARIZEMATCH Per tracker/class/robot metrics from evaluateMatch output.
%   S = summarizeMatch(T, params) -> table with columns
%     tracker, class, robot, inRegion95, within_<r> (one per params.radii,
%     e.g. within_0p25), rmse (matched tracks only), availability,
%     region_area_median, then (appended, backward compatible):
%     inRegion95_cond, within_<r>_cond  rates among samples with a matched track
%     falseTracks          mean(live tracks of class - true targets of class)
%     region_area_recent_median  median area of live tracks updated < params.recentAge s ago
%     nees_mean            mean NEES of matched tracks
%     inRegion95_capped    inRegion95 counting only tracks with area < params.maxAreaForScore
%   Unconditional rates use ALL samples as denominator (a miss counts as failure).
%   T may be a vertical concatenation of several trackers' tables.
radii = params.radii(:)';
trk = unique(T.tracker, 'stable'); cls = unique(T.class, 'stable');
rob = unique(T.robot)';
nR = numel(radii);
R = {}; W = zeros(0, nR); Wc = zeros(0, nR);
for a = 1:numel(trk)
  for b = 1:numel(cls)
    for c = rob
      m = T.tracker == trk(a) & T.class == cls(b) & T.robot == c;
      if ~any(m), continue; end
      h = T.hasTrack(m); e = T.err(m); ar = T.area(m);
      w = T.withinR(m,:); inR = T.inRegion95(m);
      rmse = sqrt(mean(e(h).^2)); if ~any(h), rmse = NaN; end
      if any(h), condIn = mean(inR(h)); condW = mean(w(h,:),1); nm = mean(T.nees(m & T.hasTrack), 'omitnan');
      else, condIn = NaN; condW = nan(1,nR); nm = NaN; end
      R(end+1,:) = {trk(a), cls(b), c, mean(inR), rmse, mean(h), median(ar,'omitnan'), ...
          condIn, mean(T.falseTracks(m),'omitnan'), median(T.areaRecent(m),'omitnan'), ...
          nm, mean(T.inRegion95Capped(m))}; %#ok<AGROW>
      W(end+1,:) = mean(w,1); Wc(end+1,:) = condW; %#ok<AGROW>
    end
  end
end
S = table(vertcat(R{:,1}), vertcat(R{:,2}), cell2mat(R(:,3)), cell2mat(R(:,4)), ...
    cell2mat(R(:,5)), cell2mat(R(:,6)), cell2mat(R(:,7)), ...
    'VariableNames', {'tracker','class','robot','inRegion95','rmse','availability','region_area_median'});
wn = arrayfun(@(r) ['within_' radiusTag(r)], radii, 'UniformOutput', false);
for k = 1:nR, S.(wn{k}) = W(:,k); end
S = movevars(S, wn, 'After', 'inRegion95');
S.inRegion95_cond = cell2mat(R(:,8));
for k = 1:nR, S.([wn{k} '_cond']) = Wc(:,k); end
S.falseTracks = cell2mat(R(:,9));
S.region_area_recent_median = cell2mat(R(:,10));
S.nees_mean = cell2mat(R(:,11));
S.inRegion95_capped = cell2mat(R(:,12));
end
