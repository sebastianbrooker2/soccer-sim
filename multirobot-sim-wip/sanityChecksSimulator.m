% SANITYCHECKSSIMULATOR Basic geometric sanity checks for the multi-robot
%   ball-detection simulator, run stand-alone (not through the
%   SIMULATEGROUNDTRUTH/GENERATEDETECTIONS pipeline, so each check
%   exercises the underlying geometry directly and unambiguously):
%
%    1) a noiseless bearing points exactly at the ball;
%    2) a ball behind the camera is never detected;
%    3) with zero noise and a perfect pose, rays from two robots
%       intersect exactly at the ball.

params = simParams();
tol = 1e-9;

%% 1) Noiseless bearing points exactly at the ball (off-boresight case too)

pose.Position  = [1, 2];
pose.Yaw       = 0.3;
pose.Roll      = 0;
pose.Pitch     = 0;
pose.HeadYaw   = 0.1;
pose.HeadPitch = -0.05;

[rCNn, Rnc] = frames.computeCameraPose(pose, params);

relCTrue = [0.5; -0.3; 3.0];         % arbitrary point in front of the camera, off boresight
ballPos  = rCNn + Rnc * relCTrue;

intrinsics = struct('ImageWidth', params.ImageWidth, 'ImageHeight', params.ImageHeight, ...
    'fx', params.fx, 'fy', params.fy, 'cx', params.cx, 'cy', params.cy, ...
    'DistortionCoeffs', params.DistortionCoeffs);

truePixel     = projectPoint(relCTrue, intrinsics);
bearingCamera = pixelToBearing(truePixel, intrinsics);
uNn           = Rnc * bearingCamera;

expected = (ballPos - rCNn) / norm(ballPos - rCNn);
err1     = norm(uNn - expected);
assert(err1 < tol, 'Sanity check 1 FAILED: noiseless bearing does not point at the ball (err=%.3g)', err1);
fprintf('PASS 1: noiseless bearing points at the ball (error %.3g)\n', err1);

%% 2) A ball behind the camera is never detected

relCBehind    = [0; 0; -3.0];        % 3 m behind the camera, along the negative optical axis
ballPosBehind = rCNn + Rnc * relCBehind;

relN = ballPosBehind - rCNn;
relC = Rnc.' * relN;
% This is exactly the gate GENERATEDETECTIONS applies before it will ever
% build a detection ("if relC(3) > 0"); confirming it is false here means
% a ball behind the camera can never reach the detection logic at all.
assert(relC(3) < 0, 'Sanity check 2 FAILED: ball behind the camera was not recognised as such');
fprintf('PASS 2: a ball behind the camera (z_c = %.3f) is never detected\n', relC(3));

%% 3) Zero noise, perfect pose: rays from two robots intersect at the ball

ballPos3 = [1.5; 0.8; 0.0];

robotA.Position = [-3, 1];
robotB.Position = [2, -2];

[uNnA, rCNnA] = noiselessBearingToPoint(robotA.Position, ballPos3, params, intrinsics);
[uNnB, rCNnB] = noiselessBearingToPoint(robotB.Position, ballPos3, params, intrinsics);

distA = norm(cross(ballPos3 - rCNnA, uNnA));
distB = norm(cross(ballPos3 - rCNnB, uNnB));
assert(distA < tol && distB < tol, ...
    'Sanity check 3 FAILED: a ray does not pass through the ball (distA=%.3g, distB=%.3g)', distA, distB);

% Closest point between the two rays should be the ball itself.
w0 = rCNnA - rCNnB;
b  = dot(uNnA, uNnB);
d  = dot(uNnA, w0);
e  = dot(uNnB, w0);
denom = 1 - b^2;
assert(abs(denom) > 1e-6, 'Sanity check 3 FAILED: the two rays are (near-)parallel, choose different robot poses');
s = (b*e - d) / denom;
t = (e - b*d) / denom;
midpoint = ((rCNnA + s*uNnA) + (rCNnB + t*uNnB)) / 2;
errIntersect = norm(midpoint - ballPos3);
assert(errIntersect < 1e-6, 'Sanity check 3 FAILED: ray intersection is not at the ball (err=%.3g)', errIntersect);

fprintf('PASS 3: rays from two robots intersect at the ball (error %.3g)\n', errIntersect);

fprintf('\nAll simulator sanity checks passed.\n');

function [uNn, rCNn] = noiselessBearingToPoint(positionXY, targetPos, params, intrinsics)
%NOISELESSBEARINGTOPOINT Bearing (field frame) a perfectly-posed, noiseless
%   camera at POSITIONXY would report for TARGETPOS, aiming the head
%   exactly at TARGETPOS via the same look-at geometry as STEPROBOTGT.

    pose.Position = positionXY;
    pose.Yaw      = 0;
    pose.Roll     = 0;
    pose.Pitch    = 0;

    headJoint = [positionXY(:); params.HeadHeight];
    relBody   = targetPos(:) - headJoint; % body frame == field frame here (Yaw=Roll=Pitch=0)

    pose.HeadYaw   = atan2(relBody(2), relBody(1));
    pose.HeadPitch = atan2(-relBody(3), hypot(relBody(1), relBody(2)));

    [rCNn, Rnc] = frames.computeCameraPose(pose, params);

    relN = targetPos(:) - rCNn;
    relC = Rnc.' * relN;
    assert(relC(3) > 0, 'Target ended up behind the camera; pick closer/less extreme test poses');

    truePixel     = projectPoint(relC, intrinsics);
    bearingCamera = pixelToBearing(truePixel, intrinsics);
    uNn           = Rnc * bearingCamera;

end
