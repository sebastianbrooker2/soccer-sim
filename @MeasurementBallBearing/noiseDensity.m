function out = noiseDensity(obj, system)
%NOISEDENSITY Bearing noise covariance: detector tangential noise, the
%   observer's own heading uncertainty, and its position uncertainty
%   projected onto the tangential (perpendicular-to-line-of-sight)
%   direction and scaled by 1/range, all applied along the tangential
%   direction t (perpendicular to the current bearing u). The radial
%   component of ObserverCovariance is dropped entirely -- moving the
%   observer along the line of sight doesn't change the bearing at all.
%
%   Along u itself (the depth/range direction), this is NOT the tiny
%   numerical-invertibility-only regularizer it used to be: it's now a
%   genuine, if rough, ASSUMED depth uncertainty (assumedDepthStd below),
%   standing in for the kind of imprecise range cue a real detector can
%   often get for free (e.g. inferring distance from the target's known
%   size in pixels) without needing a dedicated range sensor. Pure
%   bearing-only (assumedDepthStd -> 0, i.e. genuinely no range
%   information at all) leaves the posterior's depth extent bounded only
%   by whatever the prior/process noise happened to accumulate, which
%   can look absurd; a modest fixed depth uncertainty keeps it sane
%   without pretending to a precision this detector doesn't have. It
%   does NOT scale with range here (a real apparent-size cue would
%   likely get noisier at long range; not modelled).
%
%   Heading uncertainty is NOT divided by range (a rotation error is
%   just as wrong at any distance) whereas position uncertainty IS
%   (the same absolute position error matters a lot up close, and is
%   negligible far away) -- see the class doc for the derivation.
%
%   Linearises about the current prior mean (SYSTEM.density.mean()),
%   the same point 'affine'/BFGS updates elsewhere linearise PREDICT
%   about.

mu = system.density.mean();
d  = mu(1:2) - obj.ObserverPosition;
r  = norm(d);

u = d / r;              % unit vector, observer -> estimated ball (line of sight)
t = [-u(2); u(1)];      % unit tangential vector, perpendicular to u

sigmaTangential2 = obj.DetectionNoiseStd^2 + obj.ObserverHeadingVariance ...
    + (t.' * obj.ObserverCovariance * t) / r^2;

assumedDepthStd = 1; % m -- rough assumed depth precision (e.g. from apparent target size), not a numerical fudge factor
R = sigmaTangential2 * (t*t.') + assumedDepthStd^2 * (u*u.');
R = (R + R.') / 2; % symmetrize away round-off

out = GaussianInfo.fromMoment(R);
