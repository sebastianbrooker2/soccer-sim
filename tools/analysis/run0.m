% Baseline (faithful NUbots) run helper. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params = evalParams(); params.baseline.mode='faithful';
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
rng(1);
packets = makePacketsYolo(gt, params, 'recordings/real01/yolo');
numObs = numel(params.observers);
delivered = rand(numObs, numel(packets)) >= params.linkDropProb;
nb=0; for p=packets, nb=nb+sum(strcmp({p.detections.Class},'ball')); end
fprintf('%d packets, %d ball dets\n', numel(packets), nb);
tic; joint = runSharedTracker(packets, delivered, params.tracker); toc
save('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/joint0.mat','joint','packets','delivered','-v7.3');
