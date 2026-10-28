% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params = evalParams(); gt = loadGroundTruth('eval/gt/real01.csv', 0.05); rng(1);
packets = makePacketsYolo(gt, params, 'recordings/real01/yolo');
t = [packets.time]; idx = find(t>178.7 & t<179.4);
for i = idx
    p = packets(i); fprintf('t=%.3f robot %d nDet=%d nRaw=%d', p.time, p.RobotID, numel(p.detections), numel(p.RawBoxes));
    for d = p.detections(:).', fprintf(' | %s y=[%.2f %.2f r=%.2f] rs=%.2f LQ=%d', d.Class, d.y(1), d.y(2), d.y(3), d.LastDetection.RangeStd, isfield(d.LastDetection,'LowQuality') && d.LastDetection.LowQuality); end
    fprintf('\n');
end
raw = runRawMeasurement(packets, params);
for r=1:3, e = rawEstimates(raw(r), 179.0, 'ball', params); fprintf('raw ball r%d at 179.0: %d est\n', r, numel(e)); e = rawEstimates(raw(r), 179.0, 'robot', params); fprintf('raw robot r%d at 179.0: %d est\n', r, numel(e)); end
