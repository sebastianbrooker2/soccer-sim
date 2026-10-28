% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
for mode = {'old','new'}
  o = struct('gtFiles',{{'/home/seb/soccer-sim/eval/gt/real01.csv'}},'packetSource','yolo','duration',120,'runName',['tmp_yolo_' mode{1}],'parallel',false,'seed',1);
  if strcmp(mode{1},'old'), o.params = struct('sensor', struct('yolo', struct('model', []))); end
  tic; r = runEvaluation(o); fprintf('%s: %.1f s\n', mode{1}, toc);
  s = readtable(['/home/seb/soccer-sim/eval/results/tmp_yolo_' mode{1} '/summary.csv']); disp(s(contains(s.metric,{'inRegion95','rmse','within_0p5'}),:))
end
