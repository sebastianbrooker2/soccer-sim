% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams(); params.tracker.spawnAnalytic=true; params.tracker.spawnInflate=1; params.tracker.spawnVelStd=1; params.tracker.mergeSigma=NaN; params.tracker.lostStd=2;
rng(1); gt = synthGroundTruth(60,0.05,1); pk = makePackets(gt, params);
tp = trackerParamsFill(params);
tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
for p=1:numel(pk)
  if pk(p).time>=14.2 && pk(p).time<=14.8
    tr2 = tracks; for k=find([tr2.alive]), tr2(k).system = tr2(k).system.predict(pk(p).time); end
    fprintf('--- t=%.3f obs %d pos(%.2f,%.2f) true?\n', pk(p).time, pk(p).RobotID, pk(p).ObserverPosition);
    for i=1:numel(pk(p).detections)
      d=pk(p).detections(i); if ~strcmp(d.Class,'robot'), continue, end
      m=makeMeasurement(pk(p),d); s=sprintf(' det %s r=%.1f: ', d.TrueLabel, d.y(3));
      for k=find([tr2.alive] & strcmp({tr2.Class},'robot'))
        mu=tr2(k).system.density.mean(); [d2]=gateStats(m,tr2(k).system,d.y); s=[s sprintf('%s[%s n%d (%.1f,%.1f) d2=%.1f] ', tr2(k).Name, tr2(k).TrueLabel, numel(tr2(k).queue), mu(1),mu(2), d2)];
      end
      fprintf('%s\n', s);
    end
  end
  [tracks,~]=processPacket(tracks,pk(p),tp,[]);
end
