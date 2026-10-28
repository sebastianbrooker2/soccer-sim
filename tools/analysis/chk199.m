% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params = evalParams(); params.baseline.mode='faithful';
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
rng(1); packets = makePacketsYolo(gt, params, 'recordings/real01/yolo');
numObs=3; delivered = rand(numObs, numel(packets)) >= params.linkDropProb;
joint = runSharedTracker(packets, delivered, params);
rid = arrayfun(@(p) p.RobotID, packets);
solo=cell(1,3);
for r=1:3, dm=false(numObs,numel(packets)); dm(r,rid==r)=true; sr=runSharedTracker(packets,dm,params); solo{r}=sr(r); end
save('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/chk.mat','joint','solo','packets','delivered','params','-v7.3');
fj = fastCopies(joint); fs = fastCopies([solo{:}]);
for nm = {'joint','solo'}
  if strcmp(nm{1},'joint'), cp=joint; fc=fj; else, cp=[solo{:}]; fc=fs; end
  for r=1:3
    for t=[198.9 199 199.5]
      a = trackEstimates(cp(r), t, 'ball'); b = fastEstimates(fc(r), t, 'ball');
      fprintf('%s r%d t=%.1f  trackEst:%s | fast:%s\n', nm{1}, r, t, strjoin({a.name},','), strjoin({b.name},','));
    end
    % detail of ball tracks around 199
    tr = cp(r).tracks;
    for k=find(strcmp({tr.Class},'ball'))
      if tr(k).SpawnTime<=199.5 && (isnan(tr(k).deletedAt) || tr(k).deletedAt>=190)
        lq=nan; if ~isempty(tr(k).queue), lq=tr(k).queue(end).time; end; fprintf('   %s %s spawn=%.2f alive=%d del=%.2f reason=%s nq=%d lastq=%.2f\n', nm{1}, tr(k).Name, tr(k).SpawnTime, tr(k).alive, tr(k).deletedAt, tr(k).DeleteReason, numel(tr(k).queue), lq);
      end
    end
  end
end
