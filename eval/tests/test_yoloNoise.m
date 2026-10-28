function test_yoloNoise()
%TEST_YOLONOISE Calibrated YOLO box-error model (yoloErrorModel + detectionsToPackets).
here = fileparts(mfilename('fullpath')); addpath(genpath(fileparts(here)));
m = yoloErrorModel();
cam = k1SimCamera(0.55);
pose = struct('t', [0; 1], 'robot', [1; 1], 'x', [0; 0], 'y', [0; 0], 'theta', [0; 0], ...
    'headYaw', [0; 0], 'headPitch', [0.25; 0.25], 'posStd', [0.1; 0.1]);
opt = struct('ballRadius', 0.107, 'znear', 0.05, 'zfar', 40, 'minVisible', 0, 'minBallPx', 0);
[cp, Rc] = cameraPoseK1(cam, [0; 0; 0.55], 0, 0, 0, 0, 0.25);
prm = struct('ballRadius', 0.107, 'maxRange', 30, 'model', m, 'pitchStd', deg2rad(0.7), 'headStd', deg2rad(0.5));
ds = [2 4 6 8 10]; sg = zeros(size(ds)); rr = sg; lq = false(size(ds)); w = sg;
for k = 1:numel(ds)
    o = cameraProject('ball', [ds(k); 0; 0.107], cp, Rc, cam, opt);
    b = o.full; w(k) = b(3) + 1.7;                       % detector box is ~1.7 px too large
    det = struct('t', 0.5, 'robot', 1, 'class', {{'Ball'}}, 'u_px', b(1), 'v_px', b(2), ...
        'w_px', w(k), 'h_px', b(4) + 1.7, 'conf', 0.9);
    pk = detectionsToPackets(det, cam, pose, prm);
    assert(numel(pk.detections) == 1);
    d = pk.detections(1);
    sg(k) = d.LastDetection.RangeStd; rr(k) = d.y(3); lq(k) = d.LastDetection.LowQuality;
    assert(isfield(d.LastDetection, 'BearingStd') && d.LastDetection.BearingStd > 0);
end
assert(all(diff(sg) > 0), 'range std must grow with range: %s', mat2str(sg, 3));
assert(all(abs(rr(1:3) - ds(1:3)) < 0.35*ds(1:3)), 'ranges off: %s', mat2str(rr, 3));
assert(~lq(1) && lq(end), 'small far box must be LowQuality: w=%s', mat2str(w, 3));
% bias correction: size-only range from a +1.7 px box is better than the uncorrected formula
Zraw = cam.fx*2*0.107 ./ w;
Zcor = rr; p2 = prm; p2.rangeMethod = 'size'; Zs = zeros(size(ds));
for k = 1:numel(ds)
    o = cameraProject('ball', [ds(k); 0; 0.107], cp, Rc, cam, opt); b = o.full;
    det = struct('t', 0.5, 'robot', 1, 'class', {{'Ball'}}, 'u_px', b(1), 'v_px', b(2), ...
        'w_px', b(3) + 1.7, 'h_px', b(4) + 1.7, 'conf', 0.9);
    q = detectionsToPackets(det, cam, pose, p2); Zs(k) = q.detections(1).y(3);
end
assert(abs(Zs(3) - ds(3)) < abs(Zraw(3) - ds(3)), 'corrected size range should beat the raw one');
% the logged case: 7.7x5.7 px, conf 0.63 -> low quality with a big std
det = struct('t', 0.5, 'robot', 1, 'class', {{'Ball'}}, 'u_px', 170.7, 'v_px', 130.2, ...
    'w_px', 7.74, 'h_px', 5.74, 'conf', 0.63);
pk = detectionsToPackets(det, cam, pose, prm);
assert(pk.detections(1).LastDetection.LowQuality);
fprintf('test_yoloNoise OK: sigma(2..10 m) = %s m\n', mat2str(sg, 3));
end
