function ev = gtBallEvents(gt, opts)
%GTBALLEVENTS Out-of-play / restart events derived from ground truth.
%   STAND-IN for a referee / GameController signal (external info!).
%   ev.jump    : times of hard discontinuities (speed > jumpV, default 9 m/s)
%   ev.exit    : times the ball goes outside |x|>fx or |y|>fy for >= minOut s
%   ev.restart : times those out-of-play stretches end (ball set moving again)
%   ev.all     : sorted union of jump, exit(only for stretches>=minOut) and restart
%   ev.stretch : [tStart tEnd] of the out stretches kept
if nargin < 2, opts = struct(); end
jumpV = go(opts, 'jumpV', 9); minOut = go(opts, 'minOut', 1.5);
fx = go(opts, 'fx', 7.0); fy = go(opts, 'fy', 4.5);
b = gt.ball; t = gt.t;
v = hypot(diff(b(:,1)), diff(b(:,2))) ./ diff(t);
ev.jump = t(find(v > jumpV) + 1);
if numel(ev.jump) > 1, ev.jump = ev.jump([true; diff(ev.jump) > 1]); end
out = abs(b(:,1)) > fx | abs(b(:,2)) > fy;
d = diff([0; out; 0]); s = find(d == 1); e = find(d == -1) - 1;
keep = t(e) - t(s) >= minOut;
ev.stretch = [t(s(keep)) t(e(keep))];
ev.exit = ev.stretch(:, 1); ev.restart = ev.stretch(:, 2);
ev.all = sort([ev.jump(:); ev.exit(:); ev.restart(:)]);
end
function v = go(o, f, d), if isfield(o, f), v = o.(f); else, v = d; end, end
