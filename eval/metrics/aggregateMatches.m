function A = aggregateMatches(perMatch)
%AGGREGATEMATCHES Mean and std across matches per tracker/class/metric.
%   A = aggregateMatches(perMatch): perMatch is the table with columns
%   match, tracker, class, robot, <metrics...> (summarizeMatch rows + match).
%   Within each match the metrics are first averaged over robots (omitnan);
%   then mean/std (omitnan) are taken across matches.
%   A columns: tracker, class, metric, mean, std, nMatches.
meta = {'match','tracker','class','robot'};
metrics = setdiff(perMatch.Properties.VariableNames, meta, 'stable');
trk = unique(perMatch.tracker, 'stable'); cls = unique(perMatch.class, 'stable');
mt = unique(perMatch.match)';
tk = strings(0,1); ck = strings(0,1); mk = strings(0,1);
mu = zeros(0,1); sd = zeros(0,1); nm = zeros(0,1);
for a = 1:numel(trk)
  for b = 1:numel(cls)
    for k = 1:numel(metrics)
      v = nan(numel(mt),1);
      for q = 1:numel(mt)
        m = perMatch.match == mt(q) & perMatch.tracker == trk(a) & perMatch.class == cls(b);
        if any(m), v(q) = mean(perMatch.(metrics{k})(m), 'omitnan'); end
      end
      ok = ~isnan(v);
      if ~any(perMatch.tracker == trk(a) & perMatch.class == cls(b)), continue; end
      tk(end+1,1) = trk(a); ck(end+1,1) = cls(b); mk(end+1,1) = string(metrics{k}); %#ok<AGROW>
      nm(end+1,1) = sum(ok); %#ok<AGROW>
      if any(ok)
          mu(end+1,1) = mean(v(ok)); %#ok<AGROW>
          if sum(ok) > 1, sd(end+1,1) = std(v(ok)); else, sd(end+1,1) = 0; end %#ok<AGROW>
      else
          mu(end+1,1) = NaN; sd(end+1,1) = NaN; %#ok<AGROW>
      end
    end
  end
end
A = table(tk, ck, mk, mu, sd, nm, 'VariableNames', {'tracker','class','metric','mean','std','nMatches'});
end
