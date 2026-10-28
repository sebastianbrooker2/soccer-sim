% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams(); params.tracker.spawnAnalytic=true; params.tracker.spawnInflate=1; params.tracker.spawnVelStd=1; params.tracker.mergeSigma=NaN; params.tracker.lostStd=2;
rng(1); gt = synthGroundTruth(90,0.05,1); pk = makePackets(gt, params);
tp = trackerParamsFill(params);
tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, 'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {});
evs=[]; info=[];
for p=1:numel(pk)
  nb = numel(tracks);
  % pre-state: live robot tracks mean/sd at pk time (after predict inside processPacket; approximate using last system)
  [tracks, e] = processPacket(tracks, pk(p), tp, struct('gt',gt));
  evs=[evs e];
  for q=find(strcmp({e.type},'spawn'))
     lab = e(q).trackLabel; 
     % true pos of labelled object
     k = round(pk(p).time/0.05)+1; 
     if strcmp(lab,'ball'), continue, end
     id = 3*(startsWith(lab,'opp')) + sscanf(regexprep(lab,'\D',''),'%d'); tp_ = squeeze(gt.pos(k,:,id)).';
     live = find([tracks(1:nb).alive] & strcmp({tracks(1:nb).Class},'robot'));
     dmin=inf; sdn=nan; nfu=nan; 
     for l=live, mu=tracks(l).system.density.mean(); d=norm(mu(1:2)-tp_); if d<dmin, dmin=d; Pc=tracks(l).system.density.cov(); sdn=sqrt(max(eig(Pc(1:2,1:2)))); nfu=numel(tracks(l).queue); end, end
     info=[info; pk(p).time dmin sdn nfu numel(live) e(q).d2S];
  end
end
sp = evs(strcmp({evs.type},'spawn')); fprintf('spawns %d (%.1f/min) deletes %d\n', numel(sp), numel(sp)/1.5, sum(strcmp({evs.type},'delete')));
fprintf('dist from spawn truth to nearest live track: pct 10/50/90\n'); disp(prctile(info(:,2),[10 50 90]));
fprintf('nearest track sd: pct 10/50/90\n'); disp(prctile(info(:,3),[10 50 90]));
fprintf('# live tracks at spawn: pct\n'); disp(prctile(info(:,5),[10 50 90]));
fprintf('frac spawns with nearest track within 1m %.2f\n', mean(info(:,2)<1));
de = evs(strcmp({evs.type},'delete')); reasons={de.note}; fprintf('delete notes: lost %d missed %d\n', sum(strcmp(reasons,'lost')), sum(strcmp(reasons,'missed')));
fprintf('lifetime of deleted tracks pct 10/50/90\n'); disp(prctile([de.life],[10 50 90]));
fprintf('n fusions of deleted pct\n'); disp(prctile([de.n],[10 50 90]));
