% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params=evalParams(); rng(5); gt=synthGroundTruth(150,0.05,5); pk=makePackets(gt,params);
for v={[inf 0.5 25],[10 0.5 25],[3 0.5 25],[3 1.5 25],[3 1.0 5]}
 p=params; q=v{1}; p.baseline.maxAge=q(1); p.baseline.gate=q(2); p.baseline.maxMisses=q(3);
 cb=runBaselineTracker(pk,p); T=evaluateMatch(gt,cb,@baselineEstimates,p,'b');
 fprintf('maxAge %g gate %g misses %g: ',q);
 for cl={'ball','opponent'}
  m=T.class==cl{1}; h=T.hasTrack(m); ir=T.inRegion95(m); a=T.area(m); e=T.err(m);
  fprintf('| %s in95 %.3f avail %.3f rmse %.2f medErr %.2f medArea %.2f ',cl{1},mean(ir),mean(h),sqrt(mean(e(h).^2)),median(e(h)),median(a,'omitnan'));
 end
 fprintf('\n');
end
