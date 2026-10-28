% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params=evalParams(); rng(5); gt=synthGroundTruth(150,0.05,5); pk=makePackets(gt,params);
delivered = rand(3,numel(pk))>=params.linkDropProb;
for variant=0:1
 p=params; p.tracker.spawnAnalytic=logical(variant);
 cs=runSharedTracker(pk,delivered,p);
 L=cs(1).log; ty={L.type};
 as=L(strcmp(ty,'assoc')); mg=L(strcmp(ty,'merge')); sp=L(strcmp(ty,'spawn'));
 fprintf('variant %d: spawns %d, assoc correct %.3f, merges %d correct %.3f, spawn std med %.2f err med %.2f\n',variant,numel(sp),mean([as.flag]),numel(mg),mean([mg.flag]),median([sp.life]),median([sp.aux]));
 T=evaluateMatch(gt,cs,@trackEstimates,p,'s');
 for cl={'ball','opponent'}
  m=T.class==cl{1}; h=T.hasTrack(m); ir=T.inRegion95(m); a=T.area(m); e=T.err(m);
  fprintf('  %s in95 %.3f cond %.3f avail %.3f rmse %.2f medArea %.2f\n',cl{1},mean(ir),mean(ir(h)),mean(h),sqrt(mean(e(h).^2)),median(a,'omitnan'));
 end
 cnt=[];for r=1:3, for te=1:0.5:149, cnt(end+1)=numel(trackEstimates(cs(r),te,'robot')); end,end
 fprintf('  live robot tracks mean %.2f\n',mean(cnt));
end
