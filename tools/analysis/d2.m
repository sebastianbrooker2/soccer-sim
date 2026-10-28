% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
load('d1.mat','gt','pk','params','cb','cs');
T = evaluateMatch(gt,cb,@baselineEstimates,params,'b');
m = T.class=="opponent" & T.hasTrack;
% staleness: for robot 1, each eval time, age since last update of matched track
ages=[]; areas=[]; errs=[]; inr=[];
tEval=unique(T.t);
for r=1:3
 for i=1:20:numel(tEval)
  te=tEval(i); E=baselineEstimates(cb(r),te,'robot');
  for j=1:numel(E)
    s=cb(r).tracks(strcmp({cb(r).tracks.name},E(j).name)); k=find(s.t<=te,1,'last');
    ages(end+1)=te-s.t(k); areas(end+1)=pi*5.99*sqrt(det(E(j).cov));
  end
 end
end
fprintf('n tracks live per eval ~%.2f ; age med %.2f p90 %.2f ; area med %.2f\n', numel(ages)/(3*numel(1:20:numel(tEval))), median(ages),prctile(ages,90),median(areas));
for lo=[0 0.2 1 3 10]; mk=ages>=lo; fprintf('age>=%g: n=%d med area %.2f\n',lo,sum(mk),median(areas(mk))); end
% number of tracks / robot, lifetime
for r=1:3, tr=cb(r).tracks; cl={tr.class}; fprintf('robot %d: %d tracks (%d robot, %d ball); alive end %d\n',r,numel(tr),sum(strcmp(cl,'robot')),sum(strcmp(cl,'ball')),sum([tr.alive])); end
% steady state area right after update
a0=[]; for r=1:3, for q=cb(r).tracks(strcmp({cb(r).tracks.class},'robot')), n=size(q.P,3); for k=max(2,round(n/2)):min(n,round(n/2)+50), a0(end+1)=pi*5.99*sqrt(det(q.P(1:2,1:2,k))); end, end, end
fprintf('area just after update median %.3f\n',median(a0));
% ball: baseline errors
Tb=T(T.class=="ball"&T.hasTrack,:); fprintf('baseline ball err pcts 50/90/99: %.2f %.2f %.2f\n',prctile(Tb.err,[50 90 99]));
% teammate-track mis-match: fraction of baseline opp matches where matched estimate is closer to a teammate than to the opp
E=0;
save('d2.mat');
