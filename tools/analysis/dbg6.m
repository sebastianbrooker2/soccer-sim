% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams(); params.tracker.spawnAnalytic=true; params.tracker.spawnInflate=1; params.tracker.spawnVelStd=1; params.tracker.mergeSigma=NaN; params.tracker.lostStd=2;
rng(1); gt = synthGroundTruth(60,0.05,1); pk = makePackets(gt, params);
tp = trackerParamsFill(params);
tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
nd=0; far=0; log=[];
for p=1:numel(pk)
  for i=1:numel(pk(p).detections)
    d=pk(p).detections(i); if ~strcmp(d.Class,'robot'), continue, end
    nd=nd+1;
    lab=d.TrueLabel; id = 3*(startsWith(lab,'opp')) + sscanf(regexprep(lab,'\D',''),'%d'); k0=round(pk(p).time/0.05)+1; tpos=squeeze(gt.pos(k0,:,id)).';
    dd=inf; for k=find([tracks.alive] & strcmp({tracks.Class},'robot')), mu=tracks(k).system.density.mean(); dd=min(dd,norm(mu(1:2)-tpos)); end
    if dd>0.6, far=far+1; log=[log; pk(p).time id dd d.y(3) pk(p).RobotID]; end
  end
  [tracks,~]=processPacket(tracks,pk(p),tp,[]);
end
fprintf('robot dets %d, no live track within 0.6 m: %d\n', nd, far);
disp(log(1:min(25,end),:))
fprintf('live robot trks at end: %d\n', sum([tracks.alive]&strcmp({tracks.Class},'robot')));
rt = tracks(strcmp({tracks.Class},'robot')); fprintf('robot tracks ever %d, ball tracks %d\n', numel(rt), numel(tracks)-numel(rt));
fprintf('spawn times (first 40):\n'); disp([rt(1:min(40,end)).SpawnTime]);
fprintf('lifetimes pct:\n'); lt=arrayfun(@(k) min(k.deletedAt,60)-k.SpawnTime, rt); disp(prctile(lt,[10 50 90]));
fprintf('reasons: '); disp(unique({rt.DeleteReason}));
fprintf('fusions of short-lived (<0.5s) : %g\n', median(arrayfun(@(k) numel(k.queue), rt(lt<0.5))));
