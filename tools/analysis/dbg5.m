% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams(); params.tracker.spawnAnalytic=true; params.tracker.spawnInflate=1; params.tracker.spawnVelStd=1; params.tracker.mergeSigma=NaN; params.tracker.lostStd=2;
rng(1); gt = synthGroundTruth(60,0.05,1); pk = makePackets(gt, params);
tp = trackerParamsFill(params);
tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
rows=[];
for p=1:numel(pk)
  tr2 = tracks; for k=find([tr2.alive]), tr2(k).system = tr2(k).system.predict(pk(p).time); end
  for i=1:numel(pk(p).detections)
    d=pk(p).detections(i); if ~strcmp(d.Class,'robot'), continue, end
    lab=d.TrueLabel; id = 3*(startsWith(lab,'opp')) + sscanf(regexprep(lab,'\D',''),'%d'); k0=round(pk(p).time/0.05)+1; tpos=squeeze(gt.pos(k0,:,id)).';
    m=makeMeasurement(pk(p),d); dd=inf; kk=0;
    for k=find([tr2.alive] & strcmp({tr2.Class},'robot')), mu=tr2(k).system.density.mean(); if norm(mu(1:2)-tpos)<dd, dd=norm(mu(1:2)-tpos); kk=k; end, end
    if dd<0.6
      [d2,~]=gateStats(m,tr2(kk).system,d.y);
      mu=tr2(kk).system.density.mean(); P=tr2(kk).system.density.cov();
      rows=[rows; pk(p).time dd d2 numel(tr2(kk).queue) sqrt(max(eig(P(1:2,1:2)))) d.y(3) pk(p).RobotID];
    end
  end
  [tracks,~]=processPacket(tracks,pk(p),tp,[]);
end
fprintf('n=%d  frac d2>14.2: %.4f (nominal 0.0027); >30: %.4f\n', size(rows,1), mean(rows(:,3)>14.2), mean(rows(:,3)>30));
big = rows(:,3)>14.2;
fprintf('big: median nfuse %g, median sd %.3f, median err %.3f | all: nfuse %g sd %.3f err %.3f\n', median(rows(big,4)), median(rows(big,5)), median(rows(big,2)), median(rows(:,4)), median(rows(:,5)), median(rows(:,2)));
fprintf('by robot id frac big: '); for r=1:3, fprintf('%.4f ', mean(big(rows(:,7)==r))); end; fprintf('\n');
fprintf('by range bins frac big (0-3,3-6,6-9,9+): '); e=[0 3 6 9 99]; for b=1:4, s=rows(:,6)>=e(b)&rows(:,6)<e(b+1); fprintf('%.4f(%d) ', mean(big(s)), sum(s)); end; fprintf('\n');
fprintf('mean d2 overall %.2f (3 expected), median %.2f (2.37)\n', mean(min(rows(:,3),100)), median(rows(:,3)));
