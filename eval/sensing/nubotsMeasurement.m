function [xy, ok, info] = nubotsMeasurement(box, cls, pose, cam, opt)
%NUBOTSMEASUREMENT Field-frame (x,y) measurement exactly as NUbots_K1 builds it from a YOLO box.
%   [XY, OK, INFO] = NUBOTSMEASUREMENT(BOX, CLS, POSE, CAM, OPT)
%   BOX  [u v w h conf]  box CENTRE (u,v) and size (w,h) in px (320x240 image), conf.
%   CLS  'ball' | 'robot'.
%   POSE believed camera pose (as in the packet): .x .y .theta .headYaw
%        .headPitch (effective, incl. trunk tilt) .trunkHeight (see makePacketsYolo).
%   CAM  k1SimCamera struct (fx fy cx cy width height pivot offset).
%   OPT  .confBall (0.6) .confRobot (0.2)  per-class YOLO thresholds (Yolo.yaml)
%        .quantise (true) integer box as in Yolo.cpp (left=int(cx-w/2) ...).
%   Reference (~/NUbots_K1):
%    module/vision/Yolo/src/Yolo.cpp:  box ints (l.204-208); rays via unproject (centre of
%      image = dims/2, rectilinear); ray_to_camera_space (l.241-245) intersects the ray with
%      the world ground z=0: rPWw = u*|Hwc.z/u.z| + Hwc.t  (abs(), so a ray above the horizon
%      is still placed in FRONT of the camera at the mirrored distance).
%      BALL  : CENTRE ray of the box -> ground (rBCc, Ball::PROJECTION, l.285-290). The ball
%              centre is 0.107 m above the ground, so the range is OVERESTIMATED by about
%              d*r/h_cam. Angular radius is computed but never used by the localisation.
%      ROBOT : BOTTOM-CENTRE ray of the box -> ground (rRCc, l.318). Height/size never used.
%    module/localisation/{Ball,Robot}Localisation.cpp: measurement = Hwc * rBCc / rRCc, xy,
%      constant R = 0.1 I; no vision covariance, no range/size cue, no max range / min range.
%   INFO.method = 'projection'. OK false: below class threshold, or non-finite.
if nargin < 5 || isempty(opt), opt = struct(); end
cb = fo(opt, 'confBall', 0.6); cr = fo(opt, 'confRobot', 0.2); q = fo(opt, 'quantise', true);
xy = [NaN; NaN]; ok = false; info = struct('method', 'projection', 'pixel', [NaN NaN]);
conf = 1; if numel(box) >= 5, conf = box(5); end
isBall = strcmp(cls, 'ball');
if (isBall && ~(conf > cb)) || (~isBall && ~(conf > cr)), return, end
u = box(1); v = box(2); w = box(3); h = box(4);
if q
    left = fix(u - w/2); top = fix(v - h/2); w = fix(w); h = fix(h);
else
    left = u - w/2; top = v - h/2;
end
if isBall, px = [left + w/2, top + h/2]; else, px = [left + w/2, top + h]; end
info.pixel = px;
% camera pose (same believed-pose geometry as detectionsToPackets)
p = pose.headPitch; hy = pose.headYaw; th = pose.theta;
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rzh = [cos(hy) -sin(hy) 0; sin(hy) cos(hy) 0; 0 0 1];
pb = cam.pivot(:) + Rzh*Ry*cam.offset(:);
Rb = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
camPos = [pose.x; pose.y; pose.trunkHeight] + Rb*[pb(1:2); 0];
camPos(3) = pose.trunkHeight + pb(3);
% ray in world (matches detectionsToPackets rayWorld: camera x fwd, left = -u, up = -v)
xn = (px(1) - cam.cx)/cam.fx; yn = (px(2) - cam.cy)/cam.fy;
pt = fo(cam, 'pitch', 0) + p; yaw = fo(cam, 'yaw', 0) + hy + th;
db = [cos(pt); 0; -sin(pt)] + xn*[0; -1; 0] + yn*[-sin(pt); 0; -cos(pt)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
d = Rz*db; d = d/norm(d);
if abs(d(3)) < 1e-9, return, end
lam = abs(camPos(3)/d(3));              % NUbots: std::abs(Hwc.translation().z() / uBCw.z())
xy = camPos(1:2) + lam*d(1:2);
ok = all(isfinite(xy));
end

function v = fo(s, n, d)
if isstruct(s) && isfield(s, n) && ~isempty(s.(n)), v = s.(n); else, v = d; end
end
