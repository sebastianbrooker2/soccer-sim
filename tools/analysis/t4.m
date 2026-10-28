% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
S=load('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/joint0.mat'); j=S.joint;
params=evalParams(); params.evalDt=0.5;
gt = loadGroundTruth('eval/gt/real01.csv', 0.05); gt.t=gt.t(1:2400); gt.ball=gt.ball(1:2400,:); gt.pos=gt.pos(1:2400,:,:); gt.theta=gt.theta(1:2400,:);
f=fastCopies(j); fprintf('nTracks %d %d %d\n', numel(f(1).tracks),numel(f(2).tracks),numel(f(3).tracks));
profile on; tic; T=evaluateMatch(gt,f,@fastEstimates,params,'x'); toc
profile off; p=profile('info'); [~,i]=sort([p.FunctionTable.TotalTime],'descend'); for k=1:8, fprintf('%s %.1f\n',p.FunctionTable(i(k)).FunctionName,p.FunctionTable(i(k)).TotalTime); end
