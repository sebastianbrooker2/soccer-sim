% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
function R = tune1(label, ov, kScale, cScale, seed, tmax)
% ov: struct of tracker overrides; kScale: scales model kVal (std); cScale: scales cFac (>1 part) 
addpath(genpath('/home/seb/soccer-sim'));
if nargin<5, seed=1; end
if nargin<6, tmax=400; end
params = evalParams(); params.evalDt=0.5;
fn = fieldnames(ov);
for i=1:numel(fn)
  if strcmp(fn{i},'observerPosStd'), params.sensor.observerPosStd=ov.(fn{i}); params.sensor.ObserverCovariance=ov.(fn{i})^2*eye(2);
  elseif strcmp(fn{i},'headingStd'), params.sensor.headingStd=deg2rad(ov.(fn{i}));
  elseif strcmp(fn{i},'posCorrTime'), params.sensor.posCorrTime=ov.(fn{i});
  else, params.tracker.(fn{i})=ov.(fn{i}); end
end
m = params.sensor.yolo.model;
for c={'ball','robot'}
  M=m.(c{1}); M.kVal=M.kVal*kScale(1+strcmp(c{1},'robot')); 
  M.cFac = 1+(M.cFac-1)*cScale; m.(c{1})=M;
end
params.sensor.yolo.model=m;
gt = loadGroundTruth('/home/seb/soccer-sim/eval/gt/real01.csv', 0.05);
keep = gt.t-gt.t(1)<=tmax; gt.t=gt.t(keep); gt.ball=gt.ball(keep,:); gt.pos=gt.pos(keep,:,:); gt.theta=gt.theta(keep,:);
for f={'z','roll','pitch','headYaw','headPitch'}, if isfield(gt,f{1}), gt.(f{1})=gt.(f{1})(keep,:); end, end
rng(seed);
packets = makePacketsYolo(gt, params, '/home/seb/soccer-sim/recordings/real01/yolo');
rng(seed+100);
delivered = rand(3,numel(packets))>=params.linkDropProb;
tic; copies = runSharedTracker(packets, delivered, params); tt=toc;
tic; T = evaluateMatch(gt, copies, @trackEstimates, params, label); te=toc;
half = tmax/2; hn={'all ','h1  ','h2  '};
R = struct('label',label);
fprintf('%s  (tracker %.0fs, eval %.0fs)\n', label, tt, te);
for cl={'ball','opponent'}
  for h=0:2
    m = strcmp(T.class,cl{1});
    if h==1, m = m & T.t<half; elseif h==2, m = m & T.t>=half; end
    S = T(m,:);
    ok = S.hasTrack & isfinite(S.nees);
    av = mean(S.hasTrack);
    fprintf('  %-8s %s NEES %.2f (med %.2f) inR95|track %.3f  medArea %.3f  avail %.3f  rmse %.3f medErr %.3f\n', cl{1}, ...
      hn{h+1}, mean(S.nees(ok)), median(S.nees(ok)), mean(S.inRegion95(ok)), median(S.area(ok)), av, sqrt(mean(S.err(ok).^2)), median(S.err(ok)));
  end
end
R.T = T; save(['res_' label '.mat'],'T');
end
