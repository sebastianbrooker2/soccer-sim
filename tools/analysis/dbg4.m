% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams(); params.tracker.spawnAnalytic=true; params.tracker.spawnInflate=1; params.tracker.spawnVelStd=1; params.tracker.mergeSigma=NaN; params.tracker.lostStd=2;
rng(1); gt = synthGroundTruth(60,0.05,1); pk = makePackets(gt, params);
tp = trackerParamsFill(params);
tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
cntNoGate=0; cntTaken=0; cntAll=0; nsp=0; ex=[];
for p=1:numel(pk)
  tr2 = tracks; for k=find([tr2.alive]), tr2(k).system = tr2(k).system.predict(pk(p).time); end
  nd=numel(pk(p).detections); D=inf(nd,numel(tr2));
  for i=1:nd
    d=pk(p).detections(i); m=makeMeasurement(pk(p),d);
    for k=find([tr2.alive] & strcmp({tr2.Class},d.Class))
      mu=tr2(k).system.density.mean(); if strcmp(d.Class,'robot') && norm(mu(1:2)-pk(p).ObserverPosition)<tp.selfExclusion, continue, end
      D(i,k)=gateStats(m,tr2(k).system,d.y);
    end
  end
  nb=numel(tracks);
  [tracks,e]=processPacket(tracks,pk(p),tp,[]);
  for q=find(strcmp({e.type},'spawn'))
    nsp=nsp+1;
    % which detection? names track
    kq = find(strcmp({tracks.Name}, e(q).name)); 
    if strcmp(tracks(kq).Class,'ball'), continue, end
    cntAll=cntAll+1;
    % find det by matching label
    di = find(strcmp({pk(p).detections.TrueLabel}, e(q).trackLabel),1);
    if all(D(di,:)>tp.gateThreshold), cntNoGate=cntNoGate+1; else, cntTaken=cntTaken+1; end
  end
end
fprintf('robot spawns %d: no track in gate %d, gated but taken %d\n', cntAll, cntNoGate, cntTaken);
