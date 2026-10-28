function out = cameraProject(kind, P, camPos, Rc, cam, opt)
%CAMERAPROJECT Project a ball (sphere) or robot (cylinder) into the K1 camera
%   and apply the simulator's visibility rule.
%   OUT = CAMERAPROJECT(KIND, P, CAMPOS, RC, CAM, OPT)
%     KIND 'ball' | 'robot'; P = world position 3x1 (ball: sphere centre;
%     robot: foot point on the ground). CAMPOS, RC from CAMERAPOSEK1.
%     OPT fields (defaults): ballRadius 0.107, robotHeight 0.95,
%     robotWidth 0.30, znear 0.05, zfar 11, minVisible 0.5, minBallPx 0,
%     minRobotPx 8 (bbox height).
%   OUT: visible, bbox [uc vc w h] of the clipped box (px), full [uc vc w h]
%     (unclipped), depth, ratio (clipped/unclipped area), clipped (bool).
%   Ball rule = ~/.soccer-train/sim/domain/detection_utils.py is_object_in_view:
%   depth in [znear,zfar]; axis-aligned box centre +- f*R/depth; visible iff
%   clipped-box area >= 50 % of the unclipped box area.  Robots are NOT
%   detected by the simulator; analogous assumed rule: bounding box of a
%   cylinder (radius W/2, height H) sampled at 3 heights x 8 angles, same 50 %
%   rule plus a minimum pixel height.
g = @(f, d) optOr(opt, f, d);
znear = g('znear', 0.05); zfar = g('zfar', 11); minVis = g('minVisible', 0.5);
out = struct('visible', false, 'bbox', [0 0 0 0], 'full', [0 0 0 0], ...
    'depth', NaN, 'ratio', 0, 'clipped', false);
W = cam.width; H = cam.height;
q = Rc.' * (P(:) - camPos(:));            % camera body frame: x fwd, y left, z up
depth = q(1); out.depth = depth;
if depth < znear || depth > zfar, return, end
switch kind
    case 'ball'
        R = g('ballRadius', 0.107);
        uc = cam.cx - cam.fx*q(2)/depth; vc = cam.cy - cam.fy*q(3)/depth;
        rx = cam.fx*R/depth; ry = cam.fy*R/depth;
        x0 = uc - rx; x1 = uc + rx; y0 = vc - ry; y1 = vc + ry;
        minPx = g('minBallPx', 0);
        tooSmall = 2*rx < minPx;
    otherwise
        Hh = g('robotHeight', 0.95); Wd = g('robotWidth', 0.30);
        [zz, aa] = ndgrid([0 0.5 1]*Hh, (0:7)*pi/4);
        pts = P(:) + [Wd/2*cos(aa(:)).'; Wd/2*sin(aa(:)).'; zz(:).'];
        qs = Rc.' * (pts - camPos(:));
        dd = qs(1, :); ok = dd > znear;
        if ~any(ok), return, end
        us = cam.cx - cam.fx*qs(2, ok)./dd(ok); vs = cam.cy - cam.fy*qs(3, ok)./dd(ok);
        x0 = min(us); x1 = max(us); y0 = min(vs); y1 = max(vs);
        tooSmall = (y1 - y0) < g('minRobotPx', 8);
        if ~all(ok), return, end      % part of the body behind the camera: not detectable
end
fullArea = (x1 - x0)*(y1 - y0);
cx0 = max(0, x0); cx1 = min(W, x1); cy0 = max(0, y0); cy1 = min(H, y1);
out.full = [(x0+x1)/2 (y0+y1)/2 x1-x0 y1-y0];
if cx1 <= cx0 || cy1 <= cy0 || fullArea <= 0, return, end
out.ratio = (cx1-cx0)*(cy1-cy0)/fullArea;
out.bbox = [(cx0+cx1)/2 (cy0+cy1)/2 cx1-cx0 cy1-cy0];
out.clipped = out.ratio < 1 - 1e-9;
out.visible = out.ratio >= minVis && ~tooSmall;
end

function v = optOr(s, f, d)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end
