function est = rawEstimates(copy, tEval, class, params)
%RAWESTIMATES Estimates "live" at tEval for the raw-measurement behaviour: the detections of the
%   most recent packet of this robot with time in (tEval-freshness, tEval]. freshness =
%   params.rawmeas.freshness (default 0.40 s, covers the largest frame gap). No prediction: mean is the measured point as-is.
%   Same signature/fields as trackEstimates (+ lastUpdate = packet time). class: 'ball'|'robot'|''.
%   Use with evaluateMatch: est fcn = @(c,t,cl) rawEstimates(c,t,cl,params).
if nargin < 3, class = ''; end
fresh = 0.40;   % covers the largest YOLO frame gap in real01 (median 0.15 s, p95 0.31 s, max 0.37 s); 0.25 left gaps with no estimate
if nargin >= 4 && isstruct(params) && isfield(params,'rawmeas') && isfield(params.rawmeas,'freshness'), fresh = params.rawmeas.freshness; end
est = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {}, 'lastUpdate', {});
raw = copy.raw; i = find(raw.t <= tEval, 1, 'last');
if isempty(i) || raw.t(i) <= tEval - fresh, return, end
cls = raw.cls{i};
for j = 1:numel(cls)
    if ~isempty(class) && ~strcmp(cls{j}, class), continue, end
    est(end+1) = struct('name', sprintf('raw%d', j), 'mean', raw.m{i}(:,j), 'cov', raw.C{i}(:,:,j), ...
        'class', cls{j}, 'lastUpdate', raw.t(i)); %#ok<AGROW>
end
end
