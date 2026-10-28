function T = dumpYoloCalib(outCsv, methods, detsFile)
%DUMPYOLOCALIB Per-detection table (truth vs makePacketsYolo estimate) for calibration.
%   T = DUMPYOLOCALIB(OUTCSV, METHODS, DETSFILE). Observer position noise is
%   switched off so range/bearing errors are the pure measurement error.
if nargin < 2, methods = {'size', 'ground', 'fuse'}; end
if nargin < 3, detsFile = 'recordings/real01/yolo/dets_all.csv'; end
root = fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(genpath(root));
gt = loadGroundTruth(fullfile(root, 'eval/gt/real01.csv'), 0.05);
T = table();
for m = 1:numel(methods)
    rng(1);
    p = evalParams(); p.sensor.model = 'yolo'; p.observerPosStd = 1e-9; p.sensor.observerPosStd = 1e-9;
    p.dropTeammateDetections = false; p.rangeMethod = methods{m};
    p.sensor.yolo.minConf = struct('ball', 0.1, 'robot', 0.1);
    p.sensor.yolo.maxRange = 60;
    src = struct('dets', fullfile(root, detsFile), 'frames', fullfile(root, 'recordings/real01/yolo/dets_frames.csv'));
    pk = makePacketsYolo(gt, p, src);
    rows = {};
    for q = pk
        for d = q.detections
            ld = d.LastDetection; td = d.TrueDelta; if isempty(td), td = [NaN; NaN]; end
            trueR = norm(td); trueB = atan2(td(2), td(1));
            estB = atan2(d.y(2), d.y(1));
            sd = 0; if isfield(ld, 'RangeStd'), sd = ld.RangeStd; end
            lq = 0; if isfield(ld, 'LowQuality'), lq = double(ld.LowQuality); end
            bs = NaN; if isfield(ld, 'BearingStd'), bs = ld.BearingStd; end
            rows(end+1, :) = {q.time, q.RobotID, d.Class, d.TrueLabel, ld.Pixel(1), ld.Pixel(2), ld.Pixel(3), ld.Pixel(4), ...
                ld.Conf, trueR, trueB, d.y(3), estB, sd, lq, q.LookHeading, d.TrueBox, bs}; %#ok<AGROW>
        end
    end
    t = cell2table(rows, 'VariableNames', {'t','robot','cls','label','u','v','w','h','conf','trueR','trueB','estR','estB','rstd','lowq','look','tbox','bsd'});
    t.tw = t.tbox(:,3); t.th = t.tbox(:,4); t.depth = t.tbox(:,5); t.tu = t.tbox(:,1); t.tv = t.tbox(:,2); t.tbox = [];
    t.method = repmat(methods(m), height(t), 1);
    T = [T; t]; %#ok<AGROW>
end
T.cls = string(T.cls); T.label = string(T.label); T.method = string(T.method);
if ~isempty(outCsv), writetable(T, outCsv); end
end
