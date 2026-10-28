function q = baselineQ(b, cl)
%BASELINEQ Diagonal of the per-second process noise (Q = diag(q)*dt), shared by
%   the tracker and baselineEstimates.
if strcmp(cl, 'ball'), q = [b.Qball b.Qball b.Qball b.Qball];
else, q = [b.QoppPos b.QoppPos b.QoppVel b.QoppVel]; end
end
