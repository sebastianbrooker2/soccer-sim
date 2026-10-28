% Compare tracker variants on real01 (time window / camera sensor) - ad-hoc analysis. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd)); rng(5);
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
keep = gt.t >= 300 & gt.t <= 420;
gt.t = gt.t(keep); gt.ball = gt.ball(keep,:); gt.pos = gt.pos(keep,:,:); gt.theta = gt.theta(keep,:);
for f = {'z','roll','pitch','headYaw','headPitch'}, if isfield(gt,f{1}), gt.(f{1}) = gt.(f{1})(keep,:); end, end
params = evalParams();
pk = makePacketsYolo(gt, params, 'recordings/real01/yolo');
fprintf('%d packets\n', numel(pk));
numObs = 3; delivered = rand(numObs, numel(pk)) >= params.linkDropProb;
rows = {};
cfg = {'legacy_K1bug', struct('mode','faithful','measurement','legacy','k1ProximityBug',true);
       'legacy_main', struct('mode','faithful','measurement','legacy');
       'nubots_K1bug', struct('mode','faithful','measurement','nubots','k1ProximityBug',true);
       'nubots_main', struct('mode','faithful','measurement','nubots');
       'tuned', struct('mode','tuned')};
T = table();
for i=1:size(cfg,1)
  p = params; p.baseline = cfg{i,2};
  c = runBaselineTracker(pk, p);
  T = [T; evaluateMatch(gt, c, @baselineEstimates, p, cfg{i,1})];
end
c = runSharedTracker(pk, delivered, params);
T = [T; evaluateMatch(gt, c, @trackEstimates, params, 'shared')];
S = summarizeMatch(T, params);
S = S(ismember(S.class,{'ball','opponent'}),:);
G = groupsummary(S, {'tracker','class'}, 'mean', {'inRegion95','within_0p5','rmse','availability','falseTracks','region_area_median'});
disp(G)
% per-distance-band error: measurement - truth (relative to believed obs), matched by box pixel
edges = [0 2 4 6 8 11 20]; 
E = struct('cls',{},'d',{},'nu',{},'sr',{});
for q = 1:numel(pk)
  p = pk(q); if isempty(p.detections), continue, end
  for d = 1:numel(p.detections)
    dt = p.detections(d); if any(isnan(dt.TrueDelta)), continue, end
    pix = dt.LastDetection.Pixel;
    ri = find(arrayfun(@(b) isequal(b.Box(1:4), pix), p.RawBoxes), 1); if isempty(ri), continue, end
    [xy, ok] = nubotsMeasurement(p.RawBoxes(ri).Box, dt.Class, p.CamPose, p.Cam, struct('confBall',0,'confRobot',0));
    if ~ok, continue, end
    obs = p.ObserverPosition(:);
    sr = dt.y(1:2)*dt.y(3);
    E(end+1) = struct('cls', dt.Class, 'd', norm(dt.TrueDelta), 'nu', norm((xy-obs)-dt.TrueDelta), 'sr', norm(sr-dt.TrueDelta)); %#ok<SAGROW>
  end
end
for cl = {'ball','robot'}
  m = strcmp({E.cls}, cl{1}); d=[E(m).d]; nu=[E(m).nu]; sr=[E(m).sr];
  fprintf('\n%s (matched dets %d): band | n | NUbots rms / med | SRIF rms / med\n', cl{1}, sum(m));
  for b=1:numel(edges)-1
    k = d>=edges(b)&d<edges(b+1); if ~any(k), continue, end
    fprintf('  %4.0f-%-4.0f %5d | %.2f / %.2f | %.2f / %.2f\n', edges(b),edges(b+1),sum(k), sqrt(mean(nu(k).^2)), median(nu(k)), sqrt(mean(sr(k).^2)), median(sr(k)));
  end
end
nraw = sum(arrayfun(@(p) numel(p.RawBoxes), pk)); ndet = sum(arrayfun(@(p) numel(p.detections), pk));
fprintf('raw boxes %d, SRIF-converted detections %d\n', nraw, ndet);
