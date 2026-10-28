function test_projectCamera()
%TEST_PROJECTCAMERA Synthetic-geometry tests of the K1 camera visibility model.
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', '..')));
cam = k1SimCamera(0.55);
opt = struct('zfar', 11);
% zero tilt, zero head: camera at trunk+pivot+offset, looking along +x, horizontal
[cp, Rc] = cameraPoseK1(cam, [0;0;0.55], 0, 0, 0, 0, 0);
chk(abs(cp(3) - (0.55 + 0.2479 + 0.108)) < 1e-9, 'camera height');
chk(abs(cp(1) - (0.0056 + 0.0613)) < 1e-9, 'camera x');
Z = cp(3);
% ball straight ahead, 3 m, at camera height-ish -> visible, centred
o = cameraProject('ball', [cp(1)+3; 0; Z], cp, Rc, cam, opt);
chk(o.visible && abs(o.bbox(1) - 160) < 1e-6 && abs(o.bbox(2) - 120) < 1e-6, 'ball ahead visible/centred');
chk(abs(o.bbox(3) - 2*cam.fx*0.107/3) < 1e-6, 'ball pixel width');
% behind
o = cameraProject('ball', [cp(1)-3; 0; Z], cp, Rc, cam, opt);
chk(~o.visible, 'ball behind not visible');
% beyond range
o = cameraProject('ball', [cp(1)+12; 0; Z], cp, Rc, cam, opt);
chk(~o.visible, 'ball beyond zfar not visible');
% half out of the image: place centre at the right edge u=320 -> ratio 0.5 (borderline) ;
d = 3; yLeft = -(320 - cam.cx)*d/cam.fx;     % u = cx - fx*y/d
o = cameraProject('ball', [cp(1)+d; yLeft + 0.02; Z], cp, Rc, cam, opt);   % slightly inside
chk(o.visible && o.clipped, 'ball just over half inside visible (clipped)');
o = cameraProject('ball', [cp(1)+d; yLeft - 0.02; Z], cp, Rc, cam, opt);   % slightly outside
chk(~o.visible, 'ball less than half inside not visible');
% outside FOV laterally
o = cameraProject('ball', [cp(1)+3; 3; Z], cp, Rc, cam, opt);
chk(~o.visible, 'ball outside hfov');
% head yaw 90 deg left makes the ball at +y visible
[cp2, Rc2] = cameraPoseK1(cam, [0;0;0.55], 0, 0, 0, pi/2, 0);
o = cameraProject('ball', [cp2(1); cp2(2)+3; Z], cp2, Rc2, cam, opt);
chk(o.visible && o.bbox(1) < 165 && o.bbox(1) > 155, 'head yaw turns camera');
% head pitch down: ball on the ground 3 m ahead is below centre
[cp3, Rc3] = cameraPoseK1(cam, [0;0;0.55], 0, 0, 0, 0, 0.3);
o = cameraProject('ball', [3; 0; 0.107], cp3, Rc3, cam, opt);
chk(o.visible && o.bbox(2) > 120 - 60 && o.bbox(2) < 120 + 100, 'pitched ball visible');
% robot 4 m ahead, standing: visible; behind: not; too close: partially out -> may fail
o = cameraProject('robot', [cp(1)+4; 0; 0], cp, Rc, cam, opt);
chk(o.visible && abs(o.bbox(4) - 4*0 - o.bbox(4)) < 1e-9 && o.bbox(4) > 30, 'robot ahead visible');
o = cameraProject('robot', [cp(1)-4; 0; 0], cp, Rc, cam, opt);
chk(~o.visible, 'robot behind not visible');
o = cameraProject('robot', [cp(1)+0.5; 0; 0], cp, Rc, cam, opt);
chk(~o.visible, 'robot 0.5 m: mostly out of image');
% end-to-end: makePackets camera model on a tiny scene
gt = synthScene();
params.sensor = struct('model', 'camera', 'Rate', 10, 'pixStd', 1.5, 'Range', 11, ...
    'GazeAmplitude', 0, 'fallbackHeadPitch', 0.25);
params.dropTeammateDetections = true;
rng(1);
pk = makePackets(gt, params);
nb = 0; for i = 1:numel(pk), nb = nb + sum(strcmp({pk(i).detections.Class}, 'ball')); end
chk(nb > 0, 'camera model produces ball detections');
errs = [];
for i = 1:numel(pk)
    for d = pk(i).detections
        if strcmp(d.Class, 'ball'), errs(end+1) = d.y(3) - norm(gt.ball(1,:) - gt.pos(1,:,pk(i).RobotID)); end %#ok<AGROW>
    end
end
chk(median(abs(errs)) < 0.8, sprintf('ball range error median %.2f m', median(abs(errs))));
disp('test_projectCamera: all passed');
end

function gt = synthScene()
Nt = 101; gt.t = (0:Nt-1).' * 0.05;
gt.team = [0 1 1 1 2 2 2]; gt.team = [1 1 1 2 2 2];
gt.pos = zeros(Nt, 2, 6);
gt.pos(:, :, 1) = repmat([0 0], Nt, 1);
gt.pos(:, :, 2) = repmat([-3 3], Nt, 1); gt.pos(:, :, 3) = repmat([-3 -3], Nt, 1);
gt.pos(:, :, 4) = repmat([5 0], Nt, 1); gt.pos(:, :, 5) = repmat([5 2], Nt, 1); gt.pos(:, :, 6) = repmat([5 -2], Nt, 1);
gt.theta = zeros(Nt, 6); gt.ball = repmat([3 0], Nt, 1);
gt.hasHead = false;
end

function chk(c, msg)
if ~c, error('test_projectCamera FAILED: %s', msg); end
end
