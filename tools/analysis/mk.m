% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params = evalParams(); params.baseline.mode='faithful';
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
rng(1); packets = makePacketsYolo(gt, params, 'recordings/real01/yolo');
numObs=3; delivered = rand(numObs, numel(packets)) >= params.linkDropProb;
joint = runSharedTracker(packets, delivered, params);
rid = arrayfun(@(p) p.RobotID, packets); own=cell(1,3);
for r=1:3, dm=false(numObs,numel(packets)); dm(r,rid==r)=true; sr=runSharedTracker(packets,dm,params); own{r}=sr(r); end
own=[own{:}];
fj=fastCopies(joint); fo=fastCopies(own);
save('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/cop.mat','joint','own','packets','delivered','gt','params','-v7.3');
