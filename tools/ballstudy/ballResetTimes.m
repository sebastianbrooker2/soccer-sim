function T = ballResetTimes(E, kinds)
%BALLRESETTIMES Times for params.tracker.ballResetTimes. kinds: subset of
%   ["jump","exit","restart"] (default all).
if nargin < 2, kinds = ["jump","exit","restart"]; end
T = E.t(E.type == "ball" & ismember(E.kind, kinds));
end
