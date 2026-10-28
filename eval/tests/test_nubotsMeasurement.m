function test_nubotsMeasurement()
%TEST_NUBOTSMEASUREMENT Known-geometry round trip of the NUbots_K1 measurement model.
here = fileparts(mfilename('fullpath')); addpath(genpath(fileparts(here)));
cam = k1SimCamera(0.55);
pose = struct('x', 1.0, 'y', -2.0, 'theta', 0.4, 'headYaw', 0.2, 'headPitch', 0.35, 'trunkHeight', 0.55);
[cp, Rc] = cameraPoseK1(cam, [pose.x; pose.y; pose.trunkHeight], pose.theta, 0, 0, pose.headYaw, pose.headPitch);
opt = struct('quantise', false, 'confBall', 0, 'confRobot', 0);
% robot: foot on the ground -> exact recovery (bottom-centre of the box)
for foot = [3.0 1.5; -1.0 0.2; 4.5 -1.0].'
    P = [pose.x + 3*cos(pose.theta) + 0*foot(1); 0; 0]; %#ok<NASGU>
    Pw = [pose.x + foot(1)*cos(pose.theta) - foot(2)*sin(pose.theta); ...
          pose.y + foot(1)*sin(pose.theta) + foot(2)*cos(pose.theta); 0];
    o = cameraProject('robot', Pw, cp, Rc, cam, struct('robotHeight', 0.95, 'robotWidth', 0.3, 'znear', 0.05, 'zfar', 30));
    if ~(o.depth > 0), continue, end
    b = o.full;   % [uc vc w h]
    [xy, ok] = nubotsMeasurement([b 1], 'robot', pose, cam, opt);
    assert(ok);
    cpx = cp; %#ok<NASGU>
    % bottom of the cylinder box is the nearest ground-contact ring, not the foot centre:
    % check against the ground intersection of the box's bottom-centre ray instead
    assert(norm(xy - Pw(1:2)) < 0.35, 'robot foot recovery %.3f', norm(xy - Pw(1:2)));
end
% ball: centre ray hits ground BEYOND the ball: xy = cam + (P-cam)*H/(H-r)
Pb = [3.5; -1.0; 0.107];
Pb(1:2) = Pb(1:2) + [pose.x; pose.y];
o = cameraProject('ball', Pb, cp, Rc, cam, struct('ballRadius', 0.107, 'znear', 0.05, 'zfar', 30));
assert(o.depth > 0);
[xy, ok] = nubotsMeasurement([o.full 1], 'ball', pose, cam, opt);
exp_ = cp(1:2) + (Pb(1:2) - cp(1:2))*cp(3)/(cp(3) - 0.107);
assert(ok && norm(xy - exp_) < 1e-6, 'ball overshoot formula, err %.2e', norm(xy - exp_));
assert(norm(xy - Pb(1:2)) > 0.1, 'ball must be biased long');
% a ground-level pixel straight below recovers a ground point exactly (consistency)
Pg = [pose.x + 2.5; pose.y + 0.3; 0];
o = cameraProject('robot', Pg + [0;0;0], cp, Rc, cam, struct('robotHeight', 1e-3, 'robotWidth', 1e-3, 'znear', 0.05, 'zfar', 30));
[xy, ~] = nubotsMeasurement([o.full 1], 'robot', pose, cam, opt);
assert(norm(xy - Pg(1:2)) < 1e-3, 'ground point round trip %.2e', norm(xy - Pg(1:2)));
% confidence filters (strict >, Yolo.cpp)
[~, ok] = nubotsMeasurement([160 120 10 10 0.5], 'ball', pose, cam, struct('confBall', 0.6)); assert(~ok);
[~, ok] = nubotsMeasurement([160 120 30 60 0.5], 'robot', pose, cam, struct('confRobot', 0.2)); assert(ok);
% integer box quantisation
[xa, ~] = nubotsMeasurement([100.7 150.3 20.9 41.8 1], 'robot', pose, cam, struct('quantise', true));
[xb, ~] = nubotsMeasurement([100.7 150.3 20.9 41.8 1], 'robot', pose, cam, struct('quantise', false));
assert(all(isfinite(xa)) && norm(xa - xb) < 1.0);
fprintf('test_nubotsMeasurement OK\n');
end
