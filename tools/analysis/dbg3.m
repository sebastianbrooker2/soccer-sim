% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams(); params.tracker.spawnAnalytic=true; params.tracker.spawnInflate=1; params.tracker.spawnVelStd=1; params.tracker.mergeSigma=NaN; params.tracker.lostStd=2;
rng(1); gt = synthGroundTruth(40,0.05,1); pk = makePackets(gt, params);
tp = trackerParamsFill(params);
tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
shown=0;
for p=1:numel(pk)
  if pk(p).time<5, [tracks,~]=processPacket(tracks,pk(p),tp,[]); continue, end
  % replicate gating before processing
  tr2 = tracks; for k=find([tr2.alive]), tr2(k).system = tr2(k).system.predict(pk(p).time); end
  for i=1:numel(pk(p).detections)
    d = pk(p).detections(i); if ~strcmp(d.Class,'robot'), continue, end
    m = makeMeasurement(pk(p), d); best=inf; bk=0; 
    for k=find([tr2.alive] & strcmp({tr2.Class},'robot'))
      [d2,d2r]=gateStats(m, tr2(k).system, d.y); if d2<best, best=d2; bk=k; end
    end
    if best>tp.gateThreshold && shown<10
      % nearest-by-position track
      lab=d.TrueLabel; id = 3*(startsWith(lab,'opp')) + sscanf(regexprep(lab,'\D',''),'%d'); k0=round(pk(p).time/0.05)+1; tpos=squeeze(gt.pos(k0,:,id)).';
      dd=inf; kk=0; for k=find([tr2.alive] & strcmp({tr2.Class},'robot')), mu=tr2(k).system.density.mean(); if norm(mu(1:2)-tpos)<dd, dd=norm(mu(1:2)-tpos); kk=k; end, end
      if dd<1.2
        mu=tr2(kk).system.density.mean(); P=tr2(kk).system.density.cov(); [h,H]=m.predict(mu,tr2(kk).system); R=m.noiseDensity(tr2(kk).system).cov(); Sm=H*P*H.'+R; nu=d.y-h;
        fprintf('t=%.2f det %s r=%.2f  nearest trk %s dist %.2f nfuse %d misses %d\n nu=%s\n diag(S)=%s diag(R)=%s eig P pos=%s\n d2=%.1f\n', pk(p).time, lab, d.y(3), tr2(kk).TrueLabel, dd, numel(tr2(kk).queue), tr2(kk).misses, mat2str(nu',3), mat2str(diag(Sm)',3), mat2str(diag(R)',3), mat2str(eig(P(1:2,1:2))',3), nu.'*(Sm\nu));
        shown=shown+1;
      end
    end
  end
  [tracks,~]=processPacket(tracks,pk(p),tp,[]);
end
