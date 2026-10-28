function est = baselineEstimates(copy, tEval, class, params)
%BASELINEESTIMATES Live baseline tracks at tEval. class optional: 'ball' | 'robot'.
%   Tracks are predicted from the latest stored state at or before tEval with
%   the same Q as the tracker (baselineQ), except 'hold' tracks (faithful
%   NUbots ball: the filter only predicts when a measurement arrives, so the
%   last emitted state is returned as is). params optional; default = the
%   parameters the tracker stored in copy.b, else baselineDefaults.
est = struct('name', {}, 'mean', {}, 'cov', {}, 'class', {});
if nargin < 3, class = ''; end
if nargin >= 4 && ~isempty(params), b = baselineDefaults(params);
elseif isfield(copy, 'b') && ~isempty(copy.b), b = copy.b;
else, b = baselineDefaults(struct()); end
tr = copy.tracks;
if isempty(tr), return; end
live = [tr.SpawnTime] <= tEval & [tr.deletedAt] > tEval;
if ~isempty(class), live = live & strcmp({tr.class}, class); end
for i = find(live)
    s = tr(i);
    k = find(s.t <= tEval, 1, 'last');
    if isempty(k), continue; end
    if s.hold
        x = s.x(:, k); P = s.P(:, :, k);
    else
        dt = tEval - s.t(k);
        F = [eye(2) dt*eye(2); zeros(2) eye(2)];
        x = F*s.x(:, k); P = F*s.P(:, :, k)*F.' + diag(baselineQ(b, s.class))*dt;
    end
    est(end+1) = struct('name', s.name, 'mean', x(1:2), 'cov', P(1:2, 1:2), 'class', s.class); %#ok<AGROW>
end
end
