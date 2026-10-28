function gt = synthTeleportGT(seed, duration, period, hold)
%SYNTHTELEPORTGT Synthetic match (synthGroundTruth) with scripted teleports:
%   every PERIOD s (default 40) the ball is set to a restart spot (corner /
%   throw-in / centre kickoff / goal-kick spot, cycling) with zero velocity and
%   held there HOLD s (default 3), then follows its original motion shifted so
%   the path is continuous (clipped to the pitch). On every 2nd event one
%   opponent (id 4..6, cycling) is also teleported to a start spot and its path shifted.
%   gt.events: table t,type,id,from_x,from_y,to_x,to_y,kind (same as gtEvents).
if nargin < 1, seed = 1; end
if nargin < 2, duration = 300; end
if nargin < 3, period = 40; end
if nargin < 4, hold = 3; end
gt = synthGroundTruth(duration, 0.05, seed);
spots = [6.9 4.4; 0 0; 0 -4.4; -6 0; -6.9 -4.4; 5 0; 0 4.4; 6 0];
t = gt.t; dt = 0.05; b0 = gt.ball; b = b0; off = [0 0];
clipP = @(p) [min(max(p(:,1), -6.9), 6.9), min(max(p(:,2), -4.4), 4.4)];
rows = {}; ne = 0; offR = zeros(2, 6);
for te = period:period:duration - hold - 5
    ne = ne + 1; i0 = find(t >= te, 1); i1 = find(t >= te + hold, 1);
    spot = spots(mod(ne - 1, size(spots, 1)) + 1, :);
    from = b(i0 - 1, :);
    seg = i0:i1; b(seg, :) = repmat(spot, numel(seg), 1);
    off = spot - b0(i1, :);
    after = i1 + 1:numel(t); b(after, :) = clipP(b0(after, :) + off);
    b0(after, :) = b(after, :); % later shifts accumulate on the new path
    rows(end+1, :) = {te, 'ball', 0, from(1), from(2), spot(1), spot(2), 'jump'}; %#ok<AGROW>
    if mod(ne, 2) == 0
        k = 4 + mod(ne/2 - 1, 3); to = [4 + 0.5*(k-4), -5]; 
        p = squeeze(gt.pos(:, :, k)); from = p(i0 - 1, :);
        sh = to - p(i0, :); idx = i0:numel(t);
        p(idx, :) = [min(max(p(idx,1) + sh(1), -6.9), 6.9), min(max(p(idx,2) + sh(2), -4.4), 4.4)];
        gt.pos(:, :, k) = p;
        rows(end+1, :) = {te, 'robot', k, from(1), from(2), to(1), to(2), 'teleport'}; %#ok<AGROW>
    end
end
gt.ball = b;
gt.events = cell2table(rows, 'VariableNames', {'t','type','id','from_x','from_y','to_x','to_y','kind'});
gt.events.type = string(gt.events.type); gt.events.kind = string(gt.events.kind);
end
