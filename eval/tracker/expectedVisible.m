function tf = expectedVisible(mu, pkt, params)
%EXPECTEDVISIBLE Would a target at estimated position MU be comfortably
%   inside this scan's FOV wedge and range? "Comfortably" = within
%   PARAMS.EXPECTEDMARGIN of the half-angle and range (the detector's
%   confidence fades toward both edges, so edge misses are normal). The
%   caller handles the sender's own track.

d = mu(1:2) - pkt.ObserverPosition;
r = norm(d);
angErr = atan2(sin(atan2(d(2), d(1)) - pkt.LookHeading), cos(atan2(d(2), d(1)) - pkt.LookHeading));

tf = r <= params.expectedMargin * pkt.Range ...
    && abs(angErr) <= params.expectedMargin * pkt.FOV / 2;
end
