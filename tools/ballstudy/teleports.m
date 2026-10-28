function ev = teleports(gt, vMax)
%TELEPORTS Ball jumps in GT inconsistent with physics (speed > vMax).
%   ev: struct array t0,t1,from,to,dist,outBefore
if nargin < 2, vMax = 8; end
b = gt.ball; t = gt.t; dt = diff(t);
d = hypot(diff(b(:,1)), diff(b(:,2)));
idx = find(d./dt > vMax);
ev = struct('t0', {}, 't1', {}, 'from', {}, 'to', {}, 'dist', {}, 'outBefore', {});
for i = idx(:)'
    ev(end+1) = struct('t0', t(i), 't1', t(i+1), 'from', b(i,:), 'to', b(i+1,:), 'dist', d(i), ...
        'outBefore', abs(b(i,1)) > 7 || abs(b(i,2)) > 4.5); %#ok<AGROW>
end
end
