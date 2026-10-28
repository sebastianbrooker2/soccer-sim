function test_yolo()
%TEST_YOLO makePacketsYolo on a synthetic stand-in detector log.
here = fileparts(mfilename('fullpath')); addpath(genpath(fileparts(here)));
rng(3);
gt = loadGroundTruth(fullfile(fileparts(here), 'gt', 'real01.csv'), 0.05);
keep = gt.t >= 300 & gt.t <= 330;
gt.t = gt.t(keep); gt.ball = gt.ball(keep, :); gt.pos = gt.pos(keep, :, :); gt.theta = gt.theta(keep, :);
for f = {'z', 'roll', 'pitch', 'headYaw', 'headPitch'}
    if isfield(gt, f{1}), gt.(f{1}) = gt.(f{1})(keep, :); end
end
d = tempname; cleanup = onCleanup(@() rmdir(d, 's')); %#ok<NASGU>
si = makeSyntheticYoloDets(gt, d, struct('fpRate', 0.2));
assert(si.ball > 0 && si.robot > 0 && si.falsePositives > 0);
p = evalParams(); p.sensor.model = 'yolo';
p.sensor.yolo.minConf = struct('ball', 0.0, 'robot', 0.0);
[pk, info] = makePacketsYolo(gt, p, d);
assert(numel(pk) == si.frames, 'one packet per processed frame (incl. empty)');
assert(issorted([pk.time]));
n = arrayfun(@(q) numel(q.detections), pk);
assert(any(n == 0) && any(n > 0));
for q = pk(1:20:end)
    assert(q.FOV > 1 && q.Range > 0 && isfinite(q.LookHeading) && numel(q.ObserverPosition) == 2);
    for dd = q.detections
        assert(numel(dd.y) == 3 && abs(norm(dd.y(1:2)) - 1) < 1e-9 && dd.y(3) > 0);
    end
end
lab = arrayfun(@(q) {q.detections.TrueLabel}, pk, 'UniformOutput', false); lab = [lab{:}];
assert(any(strcmp(lab, 'ball')) && any(strncmp(lab, 'opp', 3)) && any(strcmp(lab, 'fp')));
assert(~any(strncmp(lab, 'team', 4)) || mean(strncmp(lab, 'team', 4)) < 0.05, 'teammates dropped');
% teammate drop off -> teammates pass through
p2 = p; p2.dropTeammateDetections = false;
pk2 = makePacketsYolo(gt, p2, d);
lab2 = arrayfun(@(q) {q.detections.TrueLabel}, pk2, 'UniformOutput', false); lab2 = [lab2{:}];
assert(sum(strncmp(lab2, 'team', 4)) > sum(strncmp(lab, 'team', 4)));
% min confidence filters
p3 = p; p3.sensor.yolo.minConf = struct('ball', 1.01, 'robot', 1.01);
pk3 = makePacketsYolo(gt, p3, d);
assert(sum(arrayfun(@(q) numel(q.detections), pk3)) == 0 && numel(pk3) == numel(pk));
fprintf('test_yolo OK: %d packets, %d dets, labels ball=%d opp=%d team=%d fp=%d, dropped=%d\n', ...
    numel(pk), sum(n), sum(strcmp(lab,'ball')), sum(strncmp(lab,'opp',3)), sum(strncmp(lab,'team',4)), ...
    sum(strcmp(lab,'fp')), sum([info.teammateDropped]));
end
