% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params = evalParams(); rng(5);
gt = synthGroundTruth(300,0.05,5);
pk = makePackets(gt,params);
fprintf('packets %d\n',numel(pk));
% detection counts by class/label
labs = {}; for i=1:numel(pk), labs=[labs {pk(i).detections.TrueLabel}]; end
[u,~,j]=unique(labs); disp([u; num2cell(accumarray(j(:),1))']);
% min ranges respected?
mb=inf; mr=inf;
for i=1:numel(pk), for d=1:numel(pk(i).detections), y=pk(i).detections(d).y; if strcmp(pk(i).detections(d).Class,'ball'), mb=min(mb,pk(i).detections(d).LastDetection.Distance); else mr=min(mr,pk(i).detections(d).LastDetection.Distance); end, end, end
fprintf('min det dist ball %.2f robot %.2f\n',mb,mr);
delivered = rand(3,numel(pk))>=params.linkDropProb;
fprintf('delivered frac %.3f (own incl would be higher)\n',mean(delivered(:)));
cb = runBaselineTracker(pk,params);
cs = runSharedTracker(pk,delivered,params);
for nm={'baseline','shared'}
  if strcmp(nm{1},'baseline'), c=cb; f=@baselineEstimates; else c=cs; f=@trackEstimates; end
  T = evaluateMatch(gt,c,f,params,nm{1});
  for cl={'ball','opponent'}
    m=T.class==cl{1}; h=T.hasTrack(m); e=T.err(m); a=T.area(m);
    fprintf('%s %s: in95 %.3f avail %.3f rmse %.2f medErr %.2f medArea %.2f within0.5 %.2f\n',nm{1},cl{1},mean(T.inRegion95(m)),mean(h),sqrt(mean(e(h).^2)),median(e(h)),median(a,'omitnan'),mean(T.withinR(m,2)));
    % NEES-ish: among hasTrack, fraction in region
    ir=T.inRegion95(m); fprintf("   cond in95|hasTrack %.3f\n", mean(ir(h)));
  end
end
save('d1.mat');
